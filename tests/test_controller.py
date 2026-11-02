import copy,csv,gzip,json,pathlib,sys,unittest
from solution import Controller
import os
DATA = pathlib.Path(os.environ.get('CORRIDOR_DATA', pathlib.Path(__file__).resolve().parent.parent / 'data'))
FIRST_PATH = DATA / '02_train/TRAIN-001/packets.ndjson.gz'
HAS_DATA = FIRST_PATH.is_file() and (DATA / '01_reference').is_dir()
if HAS_DATA:
    with gzip.open(FIRST_PATH, 'rt') as stream:
        FIRST = json.loads(next(stream))

@unittest.skipUnless(HAS_DATA, 'Set CORRIDOR_DATA or extract organizer data to data/')
class ControllerTests(unittest.TestCase):
    def make(self):return Controller(DATA/'01_reference')
    def test_snapshot(self):
        c=self.make();d=c.decide(FIRST)
        for key,n in [('state_estimates',86),('source_assessments',50),('vehicle_assessments',72),('vehicle_actions',72)]:self.assertEqual(len(d[key]),n)
    def test_schema(self):
        import jsonschema
        jsonschema.Draft202012Validator(json.loads((DATA/'04_contract/03_decision.schema.json').read_text()),format_checker=jsonschema.FormatChecker()).validate(self.make().decide(FIRST))
    def test_repeat_packet_is_idempotent(self):
        c=self.make();d=copy.deepcopy(c.decide(FIRST));self.assertEqual(d,c.decide(FIRST))
    def test_duplicate_events(self):
        p=copy.deepcopy(FIRST);p['events']*=2
        self.assertEqual(self.make().decide(FIRST),self.make().decide(p))
    def test_unknown_fields(self):
        p=copy.deepcopy(FIRST);p['ignored_field']={'untrusted':'unused'}
        for e in p['events']:e['extra']='ignored'
        self.assertEqual(self.make().decide(FIRST),self.make().decide(p))
    def test_future_received_event_is_ignored(self):
        p=copy.deepcopy(FIRST);e=copy.deepcopy(p['events'][0]);e.update(event_id='future',event_no=999999,received_time='2099-01-01T00:00:00Z',lane_count_open_estimate=0,lane_status='CLOSED');p['events'].append(e)
        self.assertEqual(self.make().decide(FIRST),self.make().decide(p))
    def test_empty_first_packet_fails_safe(self):
        p=copy.deepcopy(FIRST);p['events']=[];d=self.make().decide(p)
        self.assertTrue(all(a['motion_action']=='HOLD' for a in d['vehicle_actions']))
        self.assertTrue(all(a['state']=='UNKNOWN' for a in d['state_estimates']))
    def test_aged_data_fails_safe(self):
        c=self.make();c.decide(FIRST);p=copy.deepcopy(FIRST);p.update(events=[],packet_id='empty',decision_time='2026-09-28T08:05:00Z');d=c.decide(p)
        self.assertTrue(all(a['motion_action'] in ('HOLD','NO_ACTION') for a in d['vehicle_actions']))
    def test_old_measurement_does_not_replace_new(self):
        c=self.make();c.decide(FIRST);p=copy.deepcopy(FIRST);e=copy.deepcopy(p['events'][0]);e.update(event_id='old',event_time='2026-09-28T07:00:00Z',received_time=FIRST['decision_time'],lane_count_open_estimate=0,lane_status='CLOSED')
        p.update(packet_id='old_packet',events=[e]);c.decide(p)
        self.assertNotEqual(c.last_data[e['source_id']]['event_id'],'old')
    def test_remote_capacity(self):
        p=copy.deepcopy(FIRST)
        for e in p['events']:
            if e['event_type']=='VEHICLE_TELEMETRY':e['autonomy_state']='REMOTE_REQUESTED';e['gnss_quality']=.1
        d=self.make().decide(p);self.assertLessEqual(sum(a['remote_support_required'] for a in d['vehicle_actions']),6)
        self.assertFalse(any(a['motion_action']=='CONTINUE' for a in d['vehicle_actions']))
    def test_mass_forbidden_closed_routes_excluded(self):
        c=self.make();c.decide(FIRST);v=next(iter(c.vehicles.values())).copy();v['gross_mass_t']='100'
        self.assertTrue(all(not c.valid_edge(s,v) for s in c.segments))
        for sid in c.state:c.state[sid]['state']='CLOSED'
        v['gross_mass_t']='10';self.assertTrue(all(not c.valid_edge(s,v) for s in c.segments))
    def test_safe_stop_capacity_and_mass(self):
        import collections
        p=copy.deepcopy(FIRST)
        for e in p['events']:
            if e['event_type']=='VEHICLE_TELEMETRY':
                e.update(segment_id='S005',offset_m=100,map_age_min=5000,autonomy_state='DEGRADED')
        c=self.make();d=c.decide(p);counts=collections.Counter()
        for a in d['vehicle_actions']:
            if a['motion_action']=='SAFE_STOP':
                stop=c.stops[a['safe_stop_id']];counts[a['safe_stop_id']]+=1
                self.assertLessEqual(float(c.vehicles[a['vehicle_id']]['gross_mass_t']),float(stop['max_vehicle_mass_t']))
        self.assertGreater(sum(counts.values()),0)
        self.assertTrue(all(n<=int(c.stops[s]['capacity_vehicles']) for s,n in counts.items()))
    def test_fresh_closure_overrides_older_open_votes(self):
        p=copy.deepcopy(FIRST)
        for e in p['events']:
            if e['event_type']=='ROAD_OBSERVATION' and e['segment_id']=='S002':
                e.update(lane_status='CLOSED',lane_count_open_estimate=0,event_time=FIRST['decision_time'])
        c=self.make();c.decide(p);self.assertEqual(c.state['S002']['state'],'CLOSED')
    def test_scenario_reset_is_explicit(self):
        c=self.make();c.decide(FIRST);p=copy.deepcopy(FIRST);p.update(scenario_id='other',packet_id='other')
        with self.assertRaises(ValueError):c.decide(p)
    def test_determinism(self):self.assertEqual(self.make().decide(FIRST),self.make().decide(FIRST))

if __name__=='__main__':unittest.main()
