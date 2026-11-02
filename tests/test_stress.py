"""Adversarial input tests using the organizer's reference and one TRAIN packet."""
import copy
import os
import gzip
import json
import pathlib
import random
import unittest

from solution import Controller

DATA = pathlib.Path(os.environ.get('CORRIDOR_DATA', pathlib.Path(__file__).resolve().parent.parent / 'data'))


@unittest.skipUnless((DATA / '01_reference').is_dir() and (DATA / '02_train/TRAIN-001/packets.ndjson.gz').is_file(), 'Extract the organizer data to data/')
class InputProvenanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with gzip.open(DATA / '02_train/TRAIN-001/packets.ndjson.gz', 'rt') as stream:
            cls.packet = json.loads(next(stream))

    def decide(self, packet):
        return Controller(DATA / '01_reference').decide(packet)

    def road_state(self, packet, segment):
        return next(x['state'] for x in self.decide(packet)['state_estimates']
                    if x['segment_id'] == segment)

    def forged_road(self, source, segment):
        packet = copy.deepcopy(self.packet)
        event = copy.deepcopy(next(x for x in packet['events']
                                   if x['event_type'] == 'ROAD_OBSERVATION'))
        event.update(event_id='forged-road-' + source + '-' + segment,
                     source_id=source, segment_id=segment, event_no=999999,
                     received_time=packet['decision_time'],
                     event_time=packet['decision_time'],
                     lane_status='CLOSED', lane_count_open_estimate=0)
        packet['events'].append(event)
        return packet

    def test_unknown_source_cannot_close_road(self):
        self.assertEqual(self.road_state(self.packet, 'S002'), 'OPEN')
        self.assertEqual(self.road_state(self.forged_road('GHOST-001', 'S002'), 'S002'), 'OPEN')

    def test_detector_cannot_report_another_segment(self):
        self.assertEqual(self.road_state(self.packet, 'S014'), 'OPEN')
        self.assertEqual(self.road_state(self.forged_road('DET-001', 'S014'), 'S014'), 'OPEN')

    def test_rsu_cannot_masquerade_as_road_detector(self):
        self.assertEqual(self.road_state(self.packet, 'S002'), 'OPEN')
        self.assertEqual(self.road_state(self.forged_road('RSU-01', 'S002'), 'S002'), 'OPEN')

    def test_vehicle_id_must_match_telemetry_source(self):
        packet = copy.deepcopy(self.packet)
        event = copy.deepcopy(next(x for x in packet['events']
                                   if x['event_type'] == 'VEHICLE_TELEMETRY' and
                                   x['vehicle_id'] == 'AV-001'))
        event.update(event_id='forged-vehicle', source_id='AV-001',
                     vehicle_id='AV-002', event_no=999999,
                     received_time=packet['decision_time'],
                     event_time=packet['decision_time'],
                     gnss_quality=0, map_age_min=9999,
                     autonomy_state='DEGRADED')
        packet['events'].append(event)
        baseline = next(x for x in self.decide(self.packet)['vehicle_actions']
                        if x['vehicle_id'] == 'AV-001')
        changed = next(x for x in self.decide(packet)['vehicle_actions']
                       if x['vehicle_id'] == 'AV-001')
        self.assertEqual(changed, baseline)

    def test_valid_observation_still_changes_road(self):
        packet = self.forged_road('DET-001', 'S002')
        self.assertEqual(self.road_state(packet, 'S002'), 'CLOSED')

    def test_event_from_another_scenario_is_ignored(self):
        packet = self.forged_road('DET-001', 'S002')
        packet['events'][-1]['scenario_id'] = 'OTHER-SCENARIO'
        self.assertEqual(self.road_state(packet, 'S002'), 'OPEN')

    def test_randomly_missing_conflicting_observations_keep_actions_admissible(self):
        rng = random.Random(20260927)
        for trial in range(35):
            packet = copy.deepcopy(self.packet)
            packet['packet_id'] = f'stress-{trial}'
            packet['events'] = [e for e in packet['events'] if rng.random() < rng.uniform(.12, .95)]
            if trial % 3 == 0:
                packet['events'].append(copy.deepcopy(rng.choice(self.packet['events'])))
            if trial % 5 == 0:
                packet['events'].append(self.forged_road('GHOST-001', 'S002')['events'][-1])
            rng.shuffle(packet['events'])
            controller = Controller(DATA / '01_reference')
            decision = controller.decide(packet)
            self.assertEqual(len(decision['state_estimates']), 86)
            self.assertEqual(len(decision['source_assessments']), 50)
            self.assertEqual(len(decision['vehicle_assessments']), 72)
            self.assertEqual(len(decision['vehicle_actions']), 72)
            self.assertLessEqual(sum(a['remote_support_required'] for a in decision['vehicle_actions']), 6)
            odd = {a['vehicle_id']: a for a in decision['vehicle_assessments']}
            stop_load = {}
            for action in decision['vehicle_actions']:
                vid = action['vehicle_id']
                if odd[vid]['odd_status'] != 'COMPLIANT':
                    self.assertNotIn(action['motion_action'], ('CONTINUE', 'REROUTE', 'LIMIT_SPEED'))
                if action['motion_action'] == 'SAFE_STOP':
                    stop_id = action['safe_stop_id']
                    stop = controller.stops[stop_id]
                    segment = controller.segments[stop['segment_id']]
                    stop_load[stop_id] = stop_load.get(stop_id, 0) + 1
                    self.assertLessEqual(float(controller.vehicles[vid]['gross_mass_t']),
                                         min(float(stop['max_vehicle_mass_t']),
                                             float(segment['weight_limit_t'])))
                    self.assertNotIn(controller.state[stop['segment_id']]['state'],
                                     ('CLOSED', 'UNKNOWN'))
                    self.assertLessEqual(stop_load[stop_id], int(stop['capacity_vehicles']))
                if action['motion_action'] == 'REROUTE':
                    telemetry = controller.last_data[vid]
                    node = controller.segments[telemetry['segment_id']]['to_node']
                    for segment_id in action['route_segment_ids']:
                        segment = controller.segments[segment_id]
                        self.assertEqual(segment['from_node'], node)
                        self.assertTrue(controller.valid_edge(segment_id, controller.vehicles[vid]))
                        node = segment['to_node']
                    self.assertEqual(node, controller.hubs[telemetry['destination_hub_id']]['node_id'])


if __name__ == '__main__':
    unittest.main()
