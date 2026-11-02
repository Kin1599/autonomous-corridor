"""Small adversarial tests that do not require the organizer's data archive."""
import unittest

from solution import Controller


class SafetyTests(unittest.TestCase):
    def controller(self):
        c=Controller.__new__(Controller)
        c.segments={
            'S1': {'segment_id':'S1','from_node':'A','to_node':'B','weight_limit_t':'20',
                   'av_allowed':'1','structure':'OPEN_ROAD'},
            'S2': {'segment_id':'S2','from_node':'B','to_node':'C','weight_limit_t':'20',
                   'av_allowed':'1','structure':'OPEN_ROAD'},
        }
        c.stops={
            'near':{'segment_id':'S1','max_vehicle_mass_t':'20','capacity_vehicles':'1','offset_m':'80'},
            'behind':{'segment_id':'S1','max_vehicle_mass_t':'20','capacity_vehicles':'1','offset_m':'20'},
            'far':{'segment_id':'S2','max_vehicle_mass_t':'20','capacity_vehicles':'1'},
        }
        c.profiles={'P':{'allowed_structures':['OPEN_ROAD']}}
        c.vehicles={'V1':{'vehicle_id':'V1','odd_profile_id':'P','gross_mass_t':'10',
                          'cargo_priority':'1'}}
        c.state={'S1':{'state':'OPEN'},'S2':{'state':'OPEN'}}
        c.now=10;c.scenario=None;c.last_packet=None;c.last_result=None
        c.previous_actions={};c.recovery={};c.remote_limit=6;c.calibration={}
        c.ingest=lambda p:None
        c.source_assessments=lambda:[]
        c.estimate_segments=lambda:[]
        c.paths=lambda start,v,load:{}
        c.vehicle_assessment=lambda vid:({'vehicle_id':vid,'odd_status':'VIOLATED',
            'violation_codes':[],'confidence':.8},{'segment_id':'S1','offset_m':50},True)
        return c

    def test_stop_must_be_forward_reachable_and_within_capacity(self):
        c=self.controller();v=c.vehicles['V1'];e={'segment_id':'S1','offset_m':50}
        self.assertTrue(c.valid_stop('near',v,e,{}))
        self.assertFalse(c.valid_stop('behind',v,e,{}))
        self.assertFalse(c.valid_stop('far',v,e,{}))
        c.valid_edge=lambda sid,v,strict_weather=True: True
        self.assertTrue(c.valid_stop('far',v,e,{}, {'B':(20,[])}))
        self.assertFalse(c.valid_stop('far',v,e,{}, {'B':(120,[])}))
        c.segments['S1']['weight_limit_t']='9'
        self.assertFalse(c.valid_stop('far',v,e,{}, {'B':(20,[])}))
        c.segments['S1']['weight_limit_t']='20'
        self.assertFalse(c.valid_stop('near',v,e,{'near':1}))
        c.state['S1']['state']='CLOSED'
        self.assertFalse(c.valid_stop('near',v,e,{}))
        c.state['S1']['state']='OPEN';v['gross_mass_t']='21'
        self.assertFalse(c.valid_stop('near',v,e,{}))

    def test_zero_open_lanes_blocks_stop(self):
        c=self.controller();c.state['S1']['lanes_open_estimate']=0
        self.assertFalse(c.valid_stop('near',c.vehicles['V1'],
                                     {'segment_id':'S1','offset_m':50},{}))

    def test_wind_violation_without_code_does_not_consume_remote_support(self):
        c=self.controller()
        d=c.decide({'scenario_id':'test','packet_id':'one','decision_time':'2026-01-01T00:00:00Z'})
        a=d['vehicle_actions'][0]
        self.assertFalse(a['remote_support_required'])
        self.assertEqual(a['motion_action'],'SAFE_STOP')
        self.assertEqual(a['safe_stop_id'],'near')

    def test_remote_support_only_for_connectivity_or_localization(self):
        c=self.controller()
        c.vehicle_assessment=lambda vid:({'vehicle_id':vid,'odd_status':'VIOLATED',
            'violation_codes':['V2X'],'confidence':.8},{'segment_id':'S1','offset_m':50},True)
        d=c.decide({'scenario_id':'test','packet_id':'one','decision_time':'2026-01-01T00:00:00Z'})
        a=d['vehicle_actions'][0]
        self.assertTrue(a['remote_support_required'])
        self.assertEqual(a['motion_action'],'HOLD')

    def test_stop_capacity_with_simultaneous_violations(self):
        c=self.controller()
        c.stops={'near':c.stops['near']}
        for n in range(2,5):
            c.vehicles[f'V{n}']=dict(c.vehicles['V1'],vehicle_id=f'V{n}')
        d=c.decide({'scenario_id':'test','packet_id':'one','decision_time':'2026-01-01T00:00:00Z'})
        actions=d['vehicle_actions']
        self.assertEqual(sum(a['motion_action']=='SAFE_STOP' for a in actions),1)
        self.assertEqual(sum(a['motion_action']=='HOLD' for a in actions),3)

    def test_remote_pool_with_simultaneous_requests(self):
        c=self.controller()
        for n in range(2,10):
            c.vehicles[f'V{n}']=dict(c.vehicles['V1'],vehicle_id=f'V{n}')
        c.vehicle_assessment=lambda vid:({'vehicle_id':vid,'odd_status':'VIOLATED',
            'violation_codes':['GNSS'],'confidence':.8},{'segment_id':'S1','offset_m':50},True)
        d=c.decide({'scenario_id':'test','packet_id':'one','decision_time':'2026-01-01T00:00:00Z'})
        self.assertEqual(sum(a['remote_support_required'] for a in d['vehicle_actions']),6)
        self.assertTrue(all(a['motion_action']=='HOLD' for a in d['vehicle_actions']))

    def test_operator_request_gets_support_without_code(self):
        c=self.controller()
        c.vehicle_assessment=lambda vid:({'vehicle_id':vid,'odd_status':'COMPLIANT',
            'violation_codes':[],'confidence':.8},
            {'segment_id':'S1','offset_m':50,'autonomy_state':'REMOTE_REQUESTED'},True)
        d=c.decide({'scenario_id':'test','packet_id':'one','decision_time':'2026-01-01T00:00:00Z'})
        a=d['vehicle_actions'][0]
        self.assertEqual(a['motion_action'],'HOLD')
        self.assertTrue(a['remote_support_required'])

    def test_simultaneous_odd_stop_and_remote_limits(self):
        c=self.controller();c.stops={'near':c.stops['near']}
        for n in range(2,10):
            c.vehicles[f'V{n}']=dict(c.vehicles['V1'],vehicle_id=f'V{n}')
        def assess(vid):
            codes=['MAP_AGE'] if vid in ('V1','V2') else ['GNSS']
            return ({'vehicle_id':vid,'odd_status':'VIOLATED','violation_codes':codes,
                     'confidence':.8},{'segment_id':'S1','offset_m':50},True)
        c.vehicle_assessment=assess
        d=c.decide({'scenario_id':'test','packet_id':'one','decision_time':'2026-01-01T00:00:00Z'})
        actions=d['vehicle_actions']
        self.assertEqual(sum(a['motion_action']=='SAFE_STOP' for a in actions),1)
        self.assertEqual(sum(a['remote_support_required'] for a in actions),6)
        self.assertTrue(all(a['motion_action'] in ('HOLD','SAFE_STOP') for a in actions))


if __name__=='__main__':
    unittest.main()
