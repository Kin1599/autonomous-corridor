"""Local metrics, not a substitute for the unpublished organizer evaluator."""
import argparse,collections,csv,gzip,json,pathlib,time,sys
import numpy as np
from sklearn.metrics import f1_score,confusion_matrix
from solution import Controller,stamp

def labels(path,key):
    rows=collections.defaultdict(dict)
    with gzip.open(path,'rt') as f:
        for r in csv.DictReader(f):rows[stamp(r['timestamp'])][r[key]]=r
    return rows

def evaluate(data,scenario,output=None,validate=False,models=None):
    folder=next(data.glob('*/'+scenario));ctrl=Controller(data/'01_reference',models)
    train=(folder/'labels').exists(); schema=None
    if validate:
        import jsonschema
        schema=jsonschema.Draft202012Validator(json.loads((data/'04_contract/03_decision.schema.json').read_text()),format_checker=jsonschema.FormatChecker())
    if train:
        sl=labels(folder/'labels/01_segment_state.csv.gz','segment_id');vl=labels(folder/'labels/02_vehicle_odd.csv.gz','vehicle_id')
        faults=list(csv.DictReader(open(folder/'labels/03_source_faults.csv')))
        for f in faults:f['start']=stamp(f['start_time']);f['end']=stamp(f['end_time'])
    pairs=collections.defaultdict(list);conf=collections.defaultdict(list);counts=collections.Counter();lat=[];errors=[];safety=collections.Counter();run=collections.Counter();actions=collections.Counter();fault_pairs=[]
    stream=gzip.GzipFile(filename=str(output),mode='wb',mtime=0) if output else None
    with gzip.open(folder/'packets.ndjson.gz','rt') as f:
        for line in f:
            p=json.loads(line);start=time.perf_counter();d=ctrl.decide(p);lat.append((time.perf_counter()-start)*1000);counts['packets']+=1
            if schema:schema.validate(d)
            for name,key,refs in [('state_estimates','segment_id',ctrl.segments),('source_assessments','source_id',ctrl.sources),('vehicle_assessments','vehicle_id',ctrl.vehicles),('vehicle_actions','vehicle_id',ctrl.vehicles)]:
                ids=[r[key] for r in d[name]];assert len(ids)==len(set(ids)) and set(ids)==set(refs)
            assert d['packet_id']==p['packet_id'] and d['scenario_id']==p['scenario_id']
            if stream:stream.write((json.dumps(d,separators=(',',':'),ensure_ascii=False,allow_nan=False)+'\n').encode())
            if not train:
                actions.update(a['motion_action'] for a in d['vehicle_actions']);continue
            now=stamp(d['decision_time']);truths=sl[now];truthv=vl[now];assess={a['vehicle_id']:a for a in d['vehicle_assessments']}
            for a in d['state_estimates']:
                true=truths[a['segment_id']]['true_state'];pred=a['state'];pairs['segment'].append((true,pred));conf['segment'].append((a['confidence']-(true==pred))**2)
                if true!=pred and len(errors)<500:errors.append({'step':p['step'],'type':'segment','entity':a['segment_id'],'truth':true,'pred':pred,'features':ctrl.last_features[a['segment_id']]})
            for a in d['source_assessments']:
                ft=sorted({r['fault_type'] for r in faults if r['source_id']==a['source_id'] and r['start']<=now<r['end']})
                true='FAILED' if 'OUTAGE' in ft else ('DEGRADED' if ft else 'OK');pred=a['status']
                pairs['source'].append((true,pred));conf['source'].append((a['confidence']-(true==pred))**2);fault_pairs.append((ft,a['fault_types']))
            occupied=collections.Counter();remote=sum(a['remote_support_required'] for a in d['vehicle_actions'])
            counts['remote_peak']=max(counts['remote_peak'],remote)
            for a in d['vehicle_actions']:
                vid=a['vehicle_id'];t=truthv[vid];va=assess[vid];motion=a['motion_action'];actions[motion]+=1
                if t['active']=='1':
                    true='COMPLIANT' if t['odd_compliant']=='1' else 'VIOLATED';pred=va['odd_status'];pairs['odd'].append((true,pred));conf['odd'].append((va['confidence']-(true==pred))**2)
                    pairs['reference_action'].append((t['reference_action_class'], 'REMOTE_SUPPORT' if a['remote_support_required'] else motion))
                    if true!=pred and len(errors)<500:errors.append({'step':p['step'],'type':'odd','entity':vid,'truth':true,'pred':pred,'true_codes':t['violation_codes'],'codes':va['violation_codes'],'autonomy':ctrl.last_data.get(vid,{}).get('autonomy_state')})
                    bad=(true=='VIOLATED' and motion in ('CONTINUE','NO_ACTION') and not a['remote_support_required'])
                    run[vid]=run[vid]+1 if bad else 0
                    if run[vid]==3:safety['unsafe_continue_episodes']+=1
                if motion=='SAFE_STOP':
                    occupied[a['safe_stop_id']]+=1;stop=ctrl.stops[a['safe_stop_id']];seg=stop['segment_id'];mass=float(ctrl.vehicles[vid]['gross_mass_t'])
                    if mass>float(stop['max_vehicle_mass_t']) or mass>float(ctrl.segments[seg]['weight_limit_t']) or truths[seg]['true_state']=='CLOSED':safety['invalid_stop_steps']+=1
                if motion=='REROUTE':
                    route=a['route_segment_ids'];node=ctrl.segments[t['segment_id']]['to_node'];mass=float(ctrl.vehicles[vid]['gross_mass_t']);valid=True
                    for sid in route:
                        s=ctrl.segments[sid];valid&=s['from_node']==node and int(s['av_allowed'])==1 and mass<=float(s['weight_limit_t']) and truths[sid]['true_state']!='CLOSED';node=s['to_node']
                    v=ctrl.last_data.get(vid,ctrl.vehicles[vid]);valid&=node==ctrl.hubs[v['destination_hub_id']]['node_id'];counts['reroutes']+=1
                    if not valid:safety['invalid_route_steps']+=1
            if remote>6:safety['remote_overcapacity_steps']+=1
            if any(n>int(ctrl.stops[k]['capacity_vehicles']) for k,n in occupied.items()):safety['stop_overcapacity_steps']+=1
    if stream:stream.close()
    result={'scenario':scenario,'counts':dict(counts),'latency_ms':{'mean':float(np.mean(lat)),'p99':float(np.percentile(lat,99)),'max':max(lat)},'actions':dict(actions),'safety_proxy':dict(safety)}
    for k,v in pairs.items():
        yt,yp=zip(*v);present=sorted(set(yt));result[k]={'macro_f1_truth_present':float(f1_score(yt,yp,labels=present,average='macro',zero_division=0)),'accuracy':float(np.mean(np.array(yt)==np.array(yp))),'classes':present,'confusion':confusion_matrix(yt,yp,labels=sorted(set(yt)|set(yp))).tolist(),'confusion_labels':sorted(set(yt)|set(yp))}
    result['calibration_brier']={k:float(np.mean(v)) for k,v in conf.items()}
    if fault_pairs:
        types=sorted(set(x for a,b in fault_pairs for x in a));result['fault_type_f1']={t:float(f1_score([t in a for a,b in fault_pairs],[t in b for a,b in fault_pairs],zero_division=0)) for t in types}
    return result,errors

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data',type=pathlib.Path,required=True);p.add_argument('--scenario',nargs='+',required=True);p.add_argument('--out',type=pathlib.Path,required=True);p.add_argument('--results',action='store_true');p.add_argument('--validate',action='store_true');p.add_argument('--models');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    for sc in a.scenario:
        r,e=evaluate(a.data,sc,a.out/(sc+'.result.ndjson.gz') if a.results else None,a.validate,a.models)
        (a.out/(sc+'.metrics.json')).write_text(json.dumps(r,ensure_ascii=False,indent=2));(a.out/(sc+'.errors.json')).write_text(json.dumps(e,ensure_ascii=False,indent=2));print(json.dumps(r,ensure_ascii=False),flush=True)
