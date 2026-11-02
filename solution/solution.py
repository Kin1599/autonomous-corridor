#!/usr/bin/env python3
"""Causal corridor controller. Runtime: Python standard library only."""
import argparse, collections, csv, datetime as dt, heapq, json, math, pathlib, statistics, sys, time

def stamp(value):
    return dt.datetime.fromisoformat(value.replace('Z','+00:00')).timestamp()

def table(root, name, key):
    with open(root / name, encoding='utf-8-sig', newline='') as f:
        return {r[key]: r for r in csv.DictReader(f)}

def median(x, default=0):
    return statistics.median(x) if x else default

class Controller:
    def __init__(self, reference, model_dir=None):
        root=pathlib.Path(reference)
        self.segments=table(root,'02_network_segments.csv','segment_id')
        self.vehicles=table(root,'10_vehicles.csv','vehicle_id')
        self.sources=table(root,'13_source_registry.csv','source_id')
        self.stops=table(root,'05_safe_stops.csv','safe_stop_id')
        self.hubs=table(root,'04_hubs.csv','hub_id')
        self.nodes=table(root,'01_network_nodes.csv','node_id')
        self.weather=table(root,'08_weather_stations.csv','station_id')
        self.profiles=json.loads((root/'09_odd_profiles.json').read_text())
        self.remote_limit=int(json.loads((root/'12_remote_support_pool.json').read_text())['max_parallel_sessions'])
        self.obs={}; self.history=collections.defaultdict(lambda:collections.deque(maxlen=120))
        self.seen=collections.OrderedDict(); self.health={}; self.last_data={}
        self.last_packet=None; self.last_result=None; self.scenario=None
        self.now=0; self.begin=None; self.previous_actions={}; self.recovery={}
        self.adj=collections.defaultdict(list)
        for sid,s in self.segments.items(): self.adj[s['from_node']].append(sid)
        self.weather_groups={}; self.calibration={}; self.model_dir=pathlib.Path(model_dir or __file__).resolve()
        if self.model_dir.is_file(): self.model_dir=self.model_dir.parent
        for name,attr in [('weather_groups.json','weather_groups'),('calibration.json','calibration')]:
            p=self.model_dir/name
            if p.exists(): setattr(self,attr,json.loads(p.read_text()))
        self.last_features={}
        self.residuals=collections.defaultdict(lambda:collections.deque(maxlen=24)); self.baselines={}; self.residual_event={}

    def valid_origin(self,e):
        """Reject events inconsistent with the fixed source and network registry."""
        sid=e.get('source_id');kind=e.get('event_type')
        if kind=='VEHICLE_TELEMETRY':return sid in self.vehicles and e.get('vehicle_id')==sid
        if kind=='HUB_STATUS':return sid in self.hubs and e.get('hub_id')==sid
        ref=self.sources.get(sid)
        if not ref:return False
        source_type=ref['source_type']
        if kind=='ROAD_OBSERVATION':
            return source_type in ('ROAD_DETECTOR','VIDEO_ANALYTICS') and e.get('segment_id')==ref['coverage']
        if kind=='V2X_MESSAGE':
            seg=self.segments.get(e.get('segment_id'))
            return source_type=='RSU' and bool(seg) and seg['v2x_zone']==ref['coverage']
        if kind=='DIGITAL_TWIN_SEGMENT':
            return source_type=='DIGITAL_TWIN' and e.get('segment_id') in self.segments
        if kind=='WEATHER_OBSERVATION':return source_type=='WEATHER'
        return kind=='INFRASTRUCTURE_HEALTH'

    def ingest(self,p):
        self.now=stamp(p.get('decision_time',p.get('packet_time')))
        if self.begin is None:self.begin=self.now
        for raw in p.get('events',[]):
            sid=raw.get('source_id'); kind=raw.get('event_type')
            if not sid or not kind:continue
            if raw.get('scenario_id')!=p['scenario_id']:continue
            if not self.valid_origin(raw):continue
            try: received=stamp(raw['received_time']); event=stamp(raw['event_time'])
            except (KeyError,ValueError,TypeError):continue
            if received>self.now+0.001:continue
            eid=raw.get('event_id')
            if eid in self.seen:continue
            if eid:self.seen[eid]=None
            while len(self.seen)>100000:self.seen.popitem(last=False)
            e=dict(raw); e['_received']=received
            skew=float(e.get('source_clock_offset_ms',0))/1000
            # A large declared clock offset affects event-time ordering, not availability.
            e['_time']=min(received,event-skew if abs(skew)>1 else event)
            e['_delay']=max(0,received-event+skew)
            key=(kind,sid,e.get('segment_id',''))
            old=self.obs.get(key)
            if old and (e['_time'],e.get('event_no',0)) <= (old['_time'],old.get('event_no',0)):continue
            self.obs[key]=e
            if kind=='INFRASTRUCTURE_HEALTH':self.health[sid]=e
            else:
                latest=self.last_data.get(sid)
                if not latest or e['_time']>=latest['_time']:self.last_data[sid]=e
                if kind in ('ROAD_OBSERVATION','WEATHER_OBSERVATION','V2X_MESSAGE'):
                    self.history[sid].append(e)
        self.by_segment=collections.defaultdict(list)
        for (kind,sid,seg),e in self.obs.items():
            if seg and kind in ('ROAD_OBSERVATION','V2X_MESSAGE','DIGITAL_TWIN_SEGMENT','VEHICLE_TELEMETRY'):
                self.by_segment[seg].append(e)

    def source_assessments(self):
        out=[];self.source_state={}
        for sid,ref in self.sources.items():
            h=self.health.get(sid,{}); e=self.last_data.get(sid,{})
            period=float(ref['expected_period_sec']);age=self.now-e.get('_received',self.begin)
            fresh_health=bool(h) and self.now-h['_time']<35
            degraded=fresh_health and h.get('status')=='DEGRADED'
            faults=[]; hist=list(self.history[sid]); recent=[x for x in hist if self.now-x['_received']<=60]
            delay=median([x['_delay'] for x in recent],e.get('_delay',0))
            offset=max(abs(h.get('time_sync_offset_ms',0)),abs(e.get('source_clock_offset_ms',0)))
            if (fresh_health and h.get('status')=='NO_HEARTBEAT') or age>max(30,4*period):
                faults=['OUTAGE'];status='FAILED';conf=.97
            else:
                if offset>1000:faults.append('TIME_SKEW')
                if ref['source_type']=='DIGITAL_TWIN':
                    stale=[x for (k,s,g),x in self.obs.items() if k=='DIGITAL_TWIN_SEGMENT' and s==sid and
                           (x.get('snapshot_age_sec',0)>60 or x['_time']-stamp(x.get('source_updated_time',x['event_time']))>60)]
                    if stale or degraded:faults.append('STALE')
                if ref['source_type']=='RSU':
                    times=sorted(set(x['_received'] for x in recent)); gaps=[b-a for a,b in zip(times,times[1:])]
                    missing=(max(0,(times[-1]-times[0])/period+1-len(times))/max(1,(times[-1]-times[0])/period+1)) if len(times)>3 else 0
                    if missing>.22 and (degraded or missing>.45):faults.append('PACKET_LOSS')
                    elif delay>3:faults.append('DELAY')
                elif delay>max(3,period*.8):faults.append('DELAY')
                if ref['source_type'] in ('ROAD_DETECTOR','VIDEO_ANALYTICS') and len(recent)>=4:
                    fields=('speed_kmh','flow_vph','occupancy_pct','queue_estimate_m')
                    signature=lambda x:tuple(x.get(k) for k in fields)
                    tail=recent[-4:]
                    if len(set(signature(x) for x in tail))==1:faults.append('FREEZE')
                    else:
                        peers=[x for x in self.by_segment.get(ref['coverage'],[]) if x['event_type']=='V2X_MESSAGE' and self.now-x['_time']<20 and x.get('signature_valid',True)]
                        if peers and e and e.get('event_id')!=self.residual_event.get(sid):
                            peer=max(peers,key=lambda x:x['_time']);res=e['speed_kmh']-peer['advisory_speed_kmh']
                            self.residuals[sid].append(res);self.residual_event[sid]=e.get('event_id')
                            if sid not in self.baselines:self.baselines[sid]=res
                            elif not degraded and abs(res-self.baselines[sid])<5:
                                self.baselines[sid]=.98*self.baselines[sid]+.02*res
                        residual=median(list(self.residuals[sid])[-6:])-self.baselines.get(sid,0)
                        if not faults and (degraded or abs(residual)>7):
                            faults.append('DRIFT' if residual>7 else 'BYZANTINE')
                if ref['source_type']=='WEATHER' and degraded and not faults:faults.append('BIAS')
                if e.get('signature_valid') is False and 'BYZANTINE' not in faults:faults.append('BYZANTINE')
                status='DEGRADED' if faults or degraded else ('OK' if e else 'UNKNOWN')
                conf=.97 if fresh_health and ((status=='OK' and h.get('status')=='OK') or degraded) else .8
            trust={'OK':.96,'DEGRADED':.35,'FAILED':.03,'UNKNOWN':.2}[status]
            result={'source_id':sid,'status':status,'trust_score':trust,'confidence':conf,'fault_types':sorted(set(faults))}
            self.source_state[sid]=result;out.append(result)
        return out

    def weather_for(self,seg):
        group=self.segments[seg]['road_group']; usable=[];nearest=[]
        s=self.segments[seg];a=self.nodes[s['from_node']];b=self.nodes[s['to_node']]
        x=(float(a['x_m'])+float(b['x_m']))/2;y=(float(a['y_m'])+float(b['y_m']))/2
        for sid,w in self.weather.items():
            e=self.last_data.get(sid)
            if not e or self.now-e['_time']>45:continue
            trust=self.source_state.get(sid,{}).get('trust_score',.2)
            if trust<.5:continue
            distance=math.hypot(float(w['x_m'])-x,float(w['y_m'])-y)
            nearest.append((distance,e))
            if self.weather_groups.get(sid)==group:usable.append(e)
        if not usable:
            usable=[e for d,e in sorted(nearest,key=lambda z:z[0])[:2] if d<30000]
        if not usable:return None
        return {'visibility_m':median([e['visibility_m'] for e in usable]),
                'rain_level':round(median([e['rain_level'] for e in usable])),
                'wind_mps':median([e['wind_mps'] for e in usable])}

    def estimate_segments(self):
        result=[];self.state={};self.conditions={};self.last_features={}
        for sid,s in self.segments.items():
            votes=[]; speeds=[]; queues=[];occ=[];fresh=[]
            for e in self.by_segment[sid]:
                kind=e['event_type']; age=self.now-e['_time'];a=self.source_state.get(e['source_id'],{})
                trust=a.get('trust_score',.8)
                ttl=40 if kind=='DIGITAL_TWIN_SEGMENT' else (8 if kind=='VEHICLE_TELEMETRY' else 25)
                if age>ttl or trust<.1:continue
                if kind=='DIGITAL_TWIN_SEGMENT' and e.get('snapshot_age_sec',0)>60:continue
                fresh.append(e)
                if kind!='VEHICLE_TELEMETRY':
                    state=e.get('closure_state') or e.get('lane_status')
                    lanes=e.get('lane_count_open',e.get('lane_count_open_estimate'))
                    if not state and lanes is not None:
                        state='CLOSED' if lanes==0 else ('PARTIAL_BLOCK' if lanes<int(s['lanes']) else 'OPEN')
                    if state:
                        w=trust*math.exp(-age/25)*(1.4 if kind=='ROAD_OBSERVATION' else 1)
                        votes.append((state,w,lanes,e['_time']))
                    if 'queue_estimate_m' in e and trust>.5:queues.append(e['queue_estimate_m'])
                    if 'occupancy_pct' in e and trust>.5:occ.append(e['occupancy_pct'])
                    speed=e.get('speed_kmh',e.get('advisory_speed_kmh'))
                    if speed is not None and trust>.5:speeds.append(speed)
            counts=collections.defaultdict(float)
            for state,w,_,t in votes:counts[state]+=w
            if counts:
                state=max(counts,key=lambda st:(counts[st],st));confidence=.96
                if len(counts)>1:confidence=.7
                # An unrefuted fresh closure is a hard safety constraint.
                closures=[(t,w) for st,w,n,t in votes if st=='CLOSED' and w>.2 and self.now-t<=10]
                if closures:
                    latest=max(t for t,w in closures)
                    newer_open=sum(st=='OPEN' and t>latest and w>.5 for st,w,n,t in votes)
                    if newer_open<2:state='CLOSED';confidence=min(confidence,.9)
                lanes=min([n for st,w,n,t in votes if st==state and n is not None],default=None)
            else:state='UNKNOWN';confidence=.35;lanes=None
            speed=median(speeds,float(s['speed_limit_kmh']));queue=median(queues);occupancy=median(occ)
            ratio=speed/max(1,float(s['speed_limit_kmh']))
            # Congestion requires queue/occupancy evidence, not a slow truck alone.
            if state=='OPEN' and (queue>70 or (ratio<.6 and (occupancy>60 or queue>30))):
                state='CONGESTED';confidence=.9
            self.last_features[sid]={'speed_ratio':ratio,'queue':queue,'occupancy':occupancy,'raw':state}
            self.conditions[sid]=self.weather_for(sid)
            w=self.conditions[sid]
            trucks=[e for e in fresh if e['event_type']=='VEHICLE_TELEMETRY' and e.get('autonomy_state')=='AUTO']
            if state=='OPEN' and len(trucks)>=2 and w and w['visibility_m']>300 and w['rain_level']<2:
                ratios=[e['speed_kmh']/max(1,min(float(s['speed_limit_kmh']),float(self.vehicles[e['vehicle_id']]['nominal_max_speed_kmh']))) for e in trucks]
                if median(ratios)<.55:state='CONGESTED';confidence=.78
            self.state[sid]={'segment_id':sid,'state':state,'lanes_open_estimate':lanes,'confidence':confidence,'rationale_codes':['STATE_'+state]}
            result.append(self.state[sid])
        return result

    def vehicle_assessment(self,vid):
        v=self.vehicles[vid];e=self.last_data.get(vid);codes=[]
        if not e or self.now-e['_time']>10:
            arrived=False
            if e and e.get('segment_id') in self.segments:
                seg=self.segments[e['segment_id']];hub=self.hubs.get(e.get('destination_hub_id'),{})
                arrived=seg['to_node']==hub.get('node_id') and float(seg['length_m'])-e.get('offset_m',0)<400
            return {'vehicle_id':vid,'odd_status':'UNKNOWN','violation_codes':[],'confidence':.8},e,not arrived
        sid=e.get('segment_id');s=self.segments.get(sid)
        if not s:return {'vehicle_id':vid,'odd_status':'UNKNOWN','violation_codes':[],'confidence':.4},e,True
        profile=self.profiles[v['odd_profile_id']];w=self.conditions.get(sid)
        if w:
            if w['visibility_m']<profile['min_visibility_m']:codes.append('VISIBILITY')
            if w['rain_level']>profile['max_rain_level']:codes.append('RAIN')
        if e['gnss_quality']<profile['min_gnss_quality']:codes.append('GNSS')
        if e['map_age_min']>profile['max_map_age_min']:codes.append('MAP_AGE')
        if float(v['gross_mass_t'])>float(s['weight_limit_t']):codes.append('WEIGHT_LIMIT')
        if s['structure'] not in profile['allowed_structures']:codes.append('STRUCTURE')
        if profile['v2x_required']:
            rsus=[k for k,r in self.sources.items() if r['source_type']=='RSU' and r['coverage']==s['v2x_zone']]
            if not rsus or all(self.source_state[k]['status'] in ('FAILED','UNKNOWN') or
                              bool(set(self.source_state[k]['fault_types']) & {'PACKET_LOSS','DELAY','TIME_SKEW'}) for k in rsus):codes.append('V2X')
        # Fresh onboard degradation is a separate, independent safety observation.
        if e.get('autonomy_state') in ('DEGRADED','REMOTE_REQUESTED') and not codes:
            if e.get('autonomy_state')=='REMOTE_REQUESTED' and profile['v2x_required']:codes.append('V2X')
            elif e.get('perception_health',1)<.86:codes.append('VISIBILITY')
        wind_bad=bool(w and w['wind_mps']>profile['max_crosswind_mps'])
        status='VIOLATED' if codes or wind_bad or e.get('autonomy_state') in ('DEGRADED','REMOTE_REQUESTED') else ('COMPLIANT' if w else 'UNKNOWN')
        confidence=.93 if status!='UNKNOWN' else .5
        return {'vehicle_id':vid,'odd_status':status,'violation_codes':sorted(codes),'confidence':confidence},e,True

    def valid_edge(self,sid,v,strict_weather=True):
        s=self.segments[sid];p=self.profiles[v['odd_profile_id']]
        if self.state[sid]['state'] in ('CLOSED','UNKNOWN'):return False
        if self.state[sid].get('lanes_open_estimate')==0:return False
        if int(s['av_allowed'])!=1 or float(s['weight_limit_t'])<float(v['gross_mass_t']):return False
        if s['structure'] not in p['allowed_structures']:return False
        if strict_weather:
            w=self.conditions.get(sid)
            if w is None or w['visibility_m']<p['min_visibility_m'] or w['rain_level']>p['max_rain_level'] or w['wind_mps']>p['max_crosswind_mps']:return False
            if p['v2x_required']:
                rsus=[k for k,r in self.sources.items() if r['source_type']=='RSU' and r['coverage']==s['v2x_zone']]
                if not any(self.source_state[k]['status']=='OK' for k in rsus):return False
        return True

    def hub_accepts(self,hub_id):
        status=self.last_data.get(hub_id)
        return not (status and self.now-status['_time']<60 and
                    status.get('accepting_new_arrivals') is False)

    def valid_stop(self,stop_id,v,e,stop_load,paths=None):
        """Reserve a stop only through a short directed, admissible path."""
        stop=self.stops[stop_id];sid=stop['segment_id']
        if not e or sid not in self.segments:return False
        if self.state[sid]['state'] in ('CLOSED','UNKNOWN'):return False
        if self.state[sid].get('lanes_open_estimate')==0:return False
        s=self.segments[sid]
        if int(s['av_allowed'])!=1:return False
        mass=float(v['gross_mass_t'])
        if mass>min(float(stop['max_vehicle_mass_t']),float(s['weight_limit_t'])):return False
        if s['structure'] not in self.profiles[v['odd_profile_id']]['allowed_structures']:return False
        if stop_load.get(stop_id,0)>=int(stop['capacity_vehicles']):return False
        if sid==e.get('segment_id'):
            if 'offset_m' in stop and stop['offset_m'] not in ('',None):
                if float(stop['offset_m'])<float(e.get('offset_m',0)):return False
        else:
            current=self.segments.get(e.get('segment_id'))
            if not current or self.state[current['segment_id']]['state'] in ('CLOSED','UNKNOWN'):return False
            if self.state[current['segment_id']].get('lanes_open_estimate')==0:return False
            if int(current['av_allowed'])!=1 or mass>float(current['weight_limit_t']):return False
            if current['structure'] not in self.profiles[v['odd_profile_id']]['allowed_structures']:return False
            if paths is None or s['from_node'] not in paths or paths[s['from_node']][0]>=120:return False
            if not self.valid_edge(sid,v,strict_weather=False):return False
        return True

    def paths(self,start,v,load):
        heap=[(0,start,[])];best={start:0};out={}
        while heap:
            cost,node,path=heapq.heappop(heap)
            if cost>best[node]:continue
            out[node]=(cost,path)
            for sid in self.adj[node]:
                if not self.valid_edge(sid,v):continue
                s=self.segments[sid];speed=max(10,float(s['speed_limit_kmh']))
                factor={'OPEN':1,'CONGESTED':4,'PARTIAL_BLOCK':5}.get(self.state[sid]['state'],100)
                factor*=1+load.get(sid,0)*360/max(1,float(s['capacity_vph']))
                nxt=cost+float(s['length_m'])/speed*3.6*factor
                dest=s['to_node']
                if nxt<best.get(dest,float('inf')):
                    best[dest]=nxt;heapq.heappush(heap,(nxt,dest,path+[sid]))
        return out

    def decide(self,p):
        if p.get('packet_id')==self.last_packet:return self.last_result
        if self.scenario is not None and p['scenario_id']!=self.scenario:raise ValueError('Use a new process per scenario')
        self.scenario=p['scenario_id']; self.ingest(p)
        sources=self.source_assessments();states=self.estimate_segments()
        assessments={};telemetry={};active={}
        for vid in self.vehicles:
            assessments[vid],telemetry[vid],active[vid]=self.vehicle_assessment(vid)
        # Atomic full-snapshot allocation: reservations cannot accumulate on repeats.
        actions={};stop_load=collections.Counter();route_load=collections.Counter();remote=0
        order=sorted(self.vehicles,key=lambda vid:(int(self.vehicles[vid]['cargo_priority']),
                     0 if self.previous_actions.get(vid,{}).get('remote_support_required') else 1,vid))
        for vid in order:
            v=self.vehicles[vid];a=assessments[vid];e=telemetry[vid];codes=a['violation_codes']
            action={'vehicle_id':vid,'motion_action':'CONTINUE','remote_support_required':False,'confidence':a['confidence'],'rationale_codes':[]}
            if not active[vid]:
                action.update(motion_action='NO_ACTION',rationale_codes=['AT_HUB_OR_INACTIVE'])
            elif a['odd_status']=='VIOLATED' or (e and e.get('autonomy_state') in ('DEGRADED','REMOTE_REQUESTED')):
                action['motion_action']='HOLD';action['rationale_codes']=['ODD_'+c for c in codes] or ['LOW_CONFIDENCE']
                # Remote help is useful for connectivity/localization, never overrides physical ODD.
                support_request=e and e.get('autonomy_state')=='REMOTE_REQUESTED'
                if ((codes and set(codes).issubset({'V2X','GNSS'})) or
                    (support_request and not codes)) and remote<self.remote_limit:
                    remote+=1;action['remote_support_required']=True
                if e and (set(codes)-{'V2X','GNSS'} or (not codes and not support_request)):
                    options=[];current=self.segments.get(e.get('segment_id'))
                    paths=self.paths(current['to_node'],v,route_load) if current else {}
                    for stop_id,stop in self.stops.items():
                        if self.valid_stop(stop_id,v,e,stop_load,paths):
                            ss=self.segments[stop['segment_id']]
                            cost=0 if stop['segment_id']==e['segment_id'] else paths[ss['from_node']][0]+float(ss['length_m'])/10
                            options.append((cost,stop_id))
                    if options:
                        _,stop_id=min(options);stop_load[stop_id]+=1
                        action.update(motion_action='SAFE_STOP',safe_stop_id=stop_id)
                self.recovery[vid]=self.now
            elif a['odd_status']=='UNKNOWN':
                action.update(motion_action='HOLD',rationale_codes=['NO_FRESH_DATA'])
            else:
                sid=e['segment_id'];s=self.segments[sid];state=self.state[sid]['state']
                if self.now-self.recovery.get(vid,-1e20)<10:
                    action.update(motion_action='HOLD',rationale_codes=['LOW_CONFIDENCE'])
                elif state in ('CLOSED','UNKNOWN') or int(s['av_allowed'])!=1:
                    action.update(motion_action='HOLD',rationale_codes=['ROUTE_AV_FORBIDDEN'] if int(s['av_allowed'])!=1 else ['STATE_'+state])
                elif state in ('CONGESTED','PARTIAL_BLOCK'):
                    action.update(motion_action='LIMIT_SPEED',speed_limit_kmh=min(30,float(s['speed_limit_kmh'])),rationale_codes=['STATE_'+state])
                    destination=e.get('destination_hub_id',v['destination_hub_id'])
                    hub=self.hubs.get(destination)
                    # A detour is issued only close to the next junction.
                    if hub and self.hub_accepts(destination) and max(50,(self.now-e['_time']+2)*e['speed_kmh']/3.6)<float(s['length_m'])-e['offset_m']<1000:
                        paths=self.paths(s['to_node'],v,route_load);target=paths.get(hub['node_id'])
                        if target and target[1]:
                            action.update(motion_action='REROUTE',route_segment_ids=target[1]);action.pop('speed_limit_kmh',None)
                            route_load.update(target[1])
                else:
                    hub=self.last_data.get(e.get('destination_hub_id',''),{})
                    if hub and self.now-hub['_time']<60 and not hub.get('accepting_new_arrivals',True) and s['to_node']==self.hubs.get(e.get('destination_hub_id'),{}).get('node_id'):
                        action.update(motion_action='HOLD',rationale_codes=['ROUTE_HUB_CAPACITY'])
                    else:action['rationale_codes']=['STATE_OPEN']
            actions[vid]=action
        self.previous_actions=actions
        result={'scenario_id':p['scenario_id'],'packet_id':p['packet_id'],'decision_time':p.get('decision_time',p.get('packet_time')),
                'state_estimates':states,'source_assessments':sources,
                'vehicle_assessments':[assessments[k] for k in self.vehicles], 'vehicle_actions':[actions[k] for k in self.vehicles]}
        for group,field in [('state_estimates','state'),('source_assessments','status'),('vehicle_assessments','odd_status')]:
            for row in result[group]:
                empirical=self.calibration.get(group,{}).get(row[field])
                if empirical is not None and row['confidence']>=.7:
                    row['confidence']=round(min(.995,max(.5,empirical),row['confidence']+.05),4)
        self.last_packet=p['packet_id'];self.last_result=result
        return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--reference',default='/data/reference')
    parser.add_argument('--model-dir',default=str(pathlib.Path(__file__).parent));args=parser.parse_args()
    controller=Controller(args.reference,args.model_dir)
    for line in sys.stdin:
        if not line.strip():continue
        start=time.perf_counter();packet=json.loads(line);result=controller.decide(packet)
        sys.stdout.write(json.dumps(result,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n');sys.stdout.flush()
        if packet.get('step',0)%60==0 or packet.get('is_final'):
            sys.stderr.write(json.dumps({'packet_id':packet['packet_id'],'latency_ms':round((time.perf_counter()-start)*1000,3)})+'\n')
if __name__=='__main__':main()
