"""Build the offline replay of one scenario (every causal step) for offline-demo/index.html.

Usage: python tools/generate_demo.py [--scenario TRAIN-004] [--out demo/data.js]
Any TRAIN or PUBLIC scenario works; labels (TRAIN only) are optional.

Every decision shown in the demo is the output of solution.Controller, fed packet by
packet exactly as in the causal run. Scenario labels are read *after* each decision
and only for on-screen annotations (event bands, detection delays, verification);
the controller never receives them. Key moments on the timeline are found in the
controller output the same way for every scenario.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
import pathlib
import sys
from collections import Counter, defaultdict

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from solution import Controller, stamp  # noqa: E402

DATA = ROOT / 'data'
SCENARIO = 'TRAIN-004'           # overridden by --scenario
OUTPUT = ROOT / 'demo' / 'data.js'  # overridden by --out

ACT = {'CONTINUE': 'C', 'LIMIT_SPEED': 'L', 'REROUTE': 'R', 'HOLD': 'H', 'SAFE_STOP': 'S', 'NO_ACTION': 'N'}
ROAD = {'OPEN': 'O', 'CONGESTED': 'C', 'PARTIAL_BLOCK': 'P', 'CLOSED': 'X', 'UNKNOWN': 'U'}
ODD = {'COMPLIANT': 'C', 'VIOLATED': 'V', 'UNKNOWN': 'U'}
SRC = {'OK': 'O', 'DEGRADED': 'D', 'FAILED': 'F', 'UNKNOWN': 'U'}
FAULTS = ['OUTAGE', 'DELAY', 'TIME_SKEW', 'FREEZE', 'DRIFT', 'PACKET_LOSS', 'BIAS', 'BYZANTINE',
          'FALSE_LANE_CLOSURE', 'STALE']
STOP_REACH_SEC = 120  # solution.valid_stop: a stop is reserved only within 120 s of directed path


def rows(path, key=None):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt', encoding='utf-8-sig', newline='') as stream:
        data = list(csv.DictReader(stream))
    return {r[key]: r for r in data} if key else data


def hhmmss(ts):
    import datetime as dt
    return dt.datetime.fromtimestamp(ts, dt.timezone.utc).strftime('%H:%M:%S')


def num(x, digits=1):
    return f'{x:.{digits}f}'.replace('.', ',')


def cars(n):
    tail = n % 100
    word = 'машин' if 11 <= tail <= 14 else {1: 'машина', 2: 'машины', 3: 'машины', 4: 'машины'}.get(n % 10, 'машин')
    return f'{n} {word}'


def nearest_stop(c, vid, e):
    """Explain why no safe stop was reserved (display only, mirrors valid_stop checks)."""
    v = c.vehicles[vid]
    current = c.segments.get(e.get('segment_id')) if e else None
    if not current:
        return None
    paths = c.paths(current['to_node'], v, {})
    mass = float(v['gross_mass_t'])
    best = None
    for stop_id, stop in c.stops.items():
        sid = stop['segment_id']
        seg = c.segments[sid]
        if mass > min(float(stop['max_vehicle_mass_t']), float(seg['weight_limit_t'])):
            continue
        if seg['structure'] not in c.profiles[v['odd_profile_id']]['allowed_structures']:
            continue
        if sid == current['segment_id']:
            cost = 0.0
        elif seg['from_node'] in paths:
            cost = paths[seg['from_node']][0]
        else:
            continue
        if best is None or cost < best[0]:
            best = (cost, stop_id)
    if best is None:
        return ('nopath', None, None)
    if best[0] >= STOP_REACH_SEC:
        return ('far', best[1], round(best[0] / 60))
    return ('busy', best[1], None)


def main():
    global SCENARIO, OUTPUT
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--scenario', default=SCENARIO)
    ap.add_argument('--out', type=pathlib.Path, default=OUTPUT)
    args = ap.parse_args()
    SCENARIO, OUTPUT = args.scenario, args.out
    folder = next((p for p in DATA.glob('*/' + SCENARIO) if (p / 'packets.ndjson.gz').exists()), None)
    assert folder, f'scenario {SCENARIO} not found under {DATA}'
    scenario_info = json.loads((folder / 'scenario.json').read_text(encoding='utf-8')) if (folder / 'scenario.json').exists() else {}
    dt = int(scenario_info.get('decision_interval_sec', 5))
    c = Controller(DATA / '01_reference')
    ref = DATA / '01_reference'
    labels = folder / 'labels'
    has_labels = (labels / '01_segment_state.csv.gz').exists()
    nodes = rows(ref / '01_network_nodes.csv', 'node_id')
    orders = rows(ref / '11_transport_orders.csv', 'vehicle_id')
    weather_stations = rows(ref / '08_weather_stations.csv', 'station_id')
    code_text = {r['code']: r['description'] for r in rows(DATA / '04_contract' / '05_rationale_codes.csv')}
    seg_ids = list(c.segments)
    seg_index = {s: i for i, s in enumerate(seg_ids)}
    veh_ids = list(c.vehicles)
    src_ids = list(c.sources)
    src_index = {s: i for i, s in enumerate(src_ids)}
    zones = sorted({s['v2x_zone'] for s in c.segments.values()})
    zone_rsus = {z: [k for k, r in c.sources.items() if r['source_type'] == 'RSU' and r['coverage'] == z] for z in zones}

    truth_seg = defaultdict(dict)
    truth_veh = defaultdict(dict)
    faults, incidents = [], []
    if has_labels:
        for r in rows(labels / '01_segment_state.csv.gz'):
            truth_seg[stamp(r['timestamp'])][r['segment_id']] = r['true_state']
        for r in rows(labels / '02_vehicle_odd.csv.gz'):
            truth_veh[stamp(r['timestamp'])][r['vehicle_id']] = r
        faults = rows(labels / '03_source_faults.csv')
        incidents = rows(labels / '04_incidents.csv')
    for f in faults + incidents:
        f['start'] = stamp(f['start_time'])
        f['end'] = stamp(f['end_time'])

    S = defaultdict(list)          # per-step arrays
    decision_table, decision_index = [], {}
    route_table, route_index = [], {}
    runs = [[] for _ in veh_ids]
    last_decision = [None] * len(veh_ids)
    history = []                   # compact per-step facts used by chapters and events
    unsafe_run = Counter()
    unsafe_episodes = 0
    t0 = None

    with gzip.open(folder / 'packets.ndjson.gz', 'rt', encoding='utf-8') as stream:
        for line in stream:
            packet = json.loads(line)
            result = c.decide(packet)
            now = stamp(packet['decision_time'])
            t0 = now if t0 is None else t0
            k = packet['step'] - 1
            actions = {a['vehicle_id']: a for a in result['vehicle_actions']}
            odd = {a['vehicle_id']: a for a in result['vehicle_assessments']}

            # roads: state string + sparse details for everything that is not OPEN
            S['roads'].append(''.join(ROAD[c.state[s]['state']] for s in seg_ids))
            details = []
            for s in seg_ids:
                st = c.state[s]
                if st['state'] == 'OPEN':
                    continue
                votes = []
                for e in c.by_segment.get(s, []):
                    kind = e['event_type']
                    if kind == 'VEHICLE_TELEMETRY':
                        continue
                    age = now - e['_time']
                    trust = c.source_state.get(e['source_id'], {}).get('trust_score', .8)
                    state = e.get('closure_state') or e.get('lane_status')
                    lanes = e.get('lane_count_open', e.get('lane_count_open_estimate'))
                    if not state and lanes is not None:
                        state = 'CLOSED' if lanes == 0 else ('PARTIAL_BLOCK' if lanes < int(c.segments[s]['lanes']) else 'OPEN')
                    # mirrors the vote weight in solution.estimate_segments (display only)
                    ttl = 40 if kind == 'DIGITAL_TWIN_SEGMENT' else 25
                    stale = kind == 'DIGITAL_TWIN_SEGMENT' and e.get('snapshot_age_sec', 0) > 60
                    used = age <= ttl and trust >= .1 and not stale and bool(state)
                    weight = trust * math.exp(-age / 25) * (1.4 if kind == 'ROAD_OBSERVATION' else 1) if used else 0
                    votes.append([e['source_id'], ROAD.get(state, '-') if state else '-', lanes, round(weight, 2),
                                  'снимок старше минуты' if stale else ('старые данные' if age > ttl else ('не доверяем' if trust < .1 else '')),
                                  round(trust, 2)])
                votes.sort()
                details.append([seg_index[s], st.get('lanes_open_estimate'), round(st['confidence'], 2), votes])
            S['roadInfo'].append(details)

            # weather used by the controller, deduplicated per step
            uniq, idx = [], []
            for s in seg_ids:
                w = c.conditions.get(s)
                key = None if w is None else [round(w['visibility_m']), int(w['rain_level']), round(w['wind_mps'], 1)]
                if key not in uniq:
                    uniq.append(key)
                idx.append(uniq.index(key))
            S['cond'].append([uniq, idx])

            v2x = ''
            for z in zones:
                bad = all(c.source_state[r]['status'] in ('FAILED', 'UNKNOWN') or
                          set(c.source_state[r]['fault_types']) & {'PACKET_LOSS', 'DELAY', 'TIME_SKEW'} for r in zone_rsus[z])
                v2x += '0' if bad else '1'
            S['v2x'].append(v2x)

            srcs = []
            for sid in src_ids:
                a = c.source_state[sid]
                if a['status'] != 'OK':
                    mask = sum(1 << FAULTS.index(f) for f in a['fault_types'])
                    srcs.append([src_index[sid], SRC[a['status']], mask, round(a['trust_score'], 2)])
            S['src'].append(srcs)

            pos, spd, gnss, mage, age = [], [], [], [], []
            for i, vid in enumerate(veh_ids):
                e = c.last_data.get(vid)
                a = actions[vid]
                o = odd[vid]
                if e and e.get('segment_id') in seg_index:
                    pos += [seg_index[e['segment_id']], round(e.get('offset_m', 0) / 10)]
                    spd.append(round(e.get('speed_kmh', 0)))
                    gnss.append(round(e.get('gnss_quality', 0) * 100))
                    mage.append(round(e.get('map_age_min', 0) * 10))
                    age.append(round(now - e['_time']))
                else:
                    pos += [-1, 0]
                    spd.append(0); gnss.append(0); mage.append(0); age.append(-1)
                route = a.get('route_segment_ids')
                r_idx = -1
                if route:
                    key = tuple(route)
                    if key not in route_index:
                        route_index[key] = len(route_table)
                        route_table.append([seg_index[x] for x in route])
                    r_idx = route_index[key]
                why = None
                physical = set(o['violation_codes']) - {'V2X', 'GNSS'}
                if a['motion_action'] == 'HOLD' and o['odd_status'] == 'VIOLATED' and physical:
                    why = nearest_stop(c, vid, e)
                decision = (ACT[a['motion_action']], tuple(a['rationale_codes']), ODD[o['odd_status']],
                            tuple(o['violation_codes']), r_idx, a.get('safe_stop_id'), int(a['remote_support_required']),
                            a.get('speed_limit_kmh'), (e or {}).get('destination_hub_id', c.vehicles[vid]['destination_hub_id']),
                            why, e.get('autonomy_state') if e else None)
                if decision not in decision_index:
                    decision_index[decision] = len(decision_table)
                    decision_table.append(list(decision[:1]) + [list(decision[1])] + [decision[2], list(decision[3])] + list(decision[4:]))
                d_idx = decision_index[decision]
                if last_decision[i] != d_idx:
                    runs[i].append([k, d_idx])
                    last_decision[i] = d_idx
            S['pos'].append(pos); S['spd'].append(spd); S['gnss'].append(gnss); S['mage'].append(mage)
            S['age'].append([[i, a] for i, a in enumerate(age) if a > 2])

            # verification against labels (display only)
            tr = truth_seg.get(now, {})
            if not has_labels:
                tr = None
            miss = [] if tr is None else [[seg_index[s], ROAD[tr[s]]] for s in seg_ids if s in tr and tr[s] != c.state[s]['state']]
            src_ok = 0
            for sid in src_ids:
                ft = {f['fault_type'] for f in faults if f['source_id'] == sid and f['start'] <= now < f['end']}
                truth = 'FAILED' if 'OUTAGE' in ft else ('DEGRADED' if ft else 'OK')
                src_ok += truth == c.source_state[sid]['status']
            tv = truth_veh.get(now, {})
            active = [v for v in veh_ids if tv.get(v, {}).get('active') == '1']
            odd_ok = sum(('COMPLIANT' if tv[v]['odd_compliant'] == '1' else 'VIOLATED') == odd[v]['odd_status'] for v in active)
            if has_labels:
                S['check'].append([len(seg_ids) - len(miss), src_ok, odd_ok, len(active)])
                S['miss'].append(miss)
            for v in active:
                bad = tv[v]['odd_compliant'] == '0' and actions[v]['motion_action'] in ('CONTINUE', 'NO_ACTION') and not actions[v]['remote_support_required']
                unsafe_run[v] = unsafe_run[v] + 1 if bad else 0
                unsafe_episodes += unsafe_run[v] == 3

            history.append({'step': packet['step'], 'now': now, 'time': packet['decision_time'][11:19],
                            'actions': {v: actions[v] for v in veh_ids}, 'odd': odd,
                            'state': {s: dict(c.state[s]) for s in seg_ids},
                            'sources': {s: dict(c.source_state[s]) for s in src_ids},
                            'cond': {s: c.conditions.get(s) for s in seg_ids},
                            'tele': {v: dict(c.last_data.get(v, {})) for v in veh_ids},
                            'details': {seg_ids[d[0]]: d for d in details},
                            'hubs': {hid: c.last_data.get(hid, {}).get('accepting_new_arrivals', True) for hid in c.hubs}})

    n = len(history)
    step_of = {h['step']: h for h in history}

    # ---- events from labels with the moment the controller first reacted -------------
    def first(pred, start, end):
        for h in history:
            if start <= h['now'] <= end + 60 and pred(h):
                return h
        return None

    events = []
    names = {'CRASH': 'ДТП', 'REROUTE_CASCADE': 'Перегрузка обхода', 'HUB_SURGE': 'Всплеск на терминале',
             'HEAVY_RAIN': 'Ливень', 'RAIN': 'Дождь', 'FOG': 'Туман', 'DEMAND_WAVE': 'Рост потока',
             'MINOR_ROADWORK': 'Дорожные работы'}
    group_ru = {'NORTH': 'северный обход', 'MAIN': 'главный ход', 'SOUTH': 'южный обход'}
    for inc in incidents:
        params = json.loads(inc['parameters_json'] or '{}')
        target = inc['target']
        if target in c.segments:
            hit = first(lambda h: h['state'][target]['state'] != 'OPEN', inc['start'], inc['end'])
            where = target
        elif target in c.hubs:
            hit = first(lambda h: h['hubs'][target] is False, inc['start'], inc['end'])
            where = target
        else:
            group = [s for s, r in c.segments.items() if r['road_group'] == target]
            if inc['event_type'] in ('HEAVY_RAIN', 'RAIN'):
                lvl = int(params.get('level', 2))
                hit = first(lambda h: any(h['cond'][s] and h['cond'][s]['rain_level'] >= lvl for s in group), inc['start'], inc['end'])
            elif inc['event_type'] == 'FOG':
                hit = first(lambda h: any(h['cond'][s] and h['cond'][s]['visibility_m'] < 150 for s in group), inc['start'], inc['end'])
            else:
                # congestion builds up gradually: measure the delay from the first labelled congestion
                onset = next((t for t in sorted(truth_seg) if inc['start'] <= t <= inc['end'] and
                              any(truth_seg[t].get(s) == 'CONGESTED' for s in group)), inc['start'])
                hit = first(lambda h: any(h['state'][s]['state'] == 'CONGESTED' for s in group), onset, inc['end'])
                inc = dict(inc, onset=onset)
            where = group_ru.get(target, target)
        events.append({'kind': 'incident', 'label': f"{names.get(inc['event_type'], inc['event_type'])} · {where}",
                       'start': round((inc['start'] - t0) / dt) + 1, 'end': round((inc['end'] - t0) / dt) + 1,
                       'detected': hit['step'] if hit else None,
                       'onset': round((inc.get('onset', inc['start']) - t0) / dt) + 1,
                       'delay': round(hit['now'] - inc.get('onset', inc['start'])) if hit else None})
    fault_ru = {'OUTAGE': 'нет сообщений', 'STALE': 'устаревшие данные', 'BYZANTINE': 'противоречивые данные',
                'DELAY': 'задержка', 'DRIFT': 'дрейф', 'FREEZE': 'залипание', 'BIAS': 'смещение',
                'PACKET_LOSS': 'потери пакетов', 'TIME_SKEW': 'сдвиг часов', 'FALSE_LANE_CLOSURE': 'ложное перекрытие'}
    for f in faults:
        sid, ft = f['source_id'], f['fault_type']
        hit = first(lambda h: ft in h['sources'][sid]['fault_types'], f['start'], f['end'])
        events.append({'kind': 'fault', 'label': f"{sid} · {fault_ru.get(ft, ft)}", 'source': sid,
                       'start': round((f['start'] - t0) / dt) + 1, 'end': round((f['end'] - t0) / dt) + 1,
                       'detected': hit['step'] if hit else None,
                       'delay': round(hit['now'] - f['start']) if hit else None})


    # ---- chapters for any scenario: key moments found in the controller output ---------
    act_ru = {'LIMIT_SPEED': 'снижает скорость', 'REROUTE': 'новый маршрут', 'HOLD': 'ждёт на месте', 'SAFE_STOP': 'едет на площадку'}
    odd_ru = {'VISIBILITY': 'видимость', 'RAIN': 'осадки', 'GNSS': 'точность GNSS', 'V2X': 'связь V2X', 'STRUCTURE': 'тип сооружения',
              'WEIGHT_LIMIT': 'масса', 'MAP_AGE': 'возраст карты'}
    kind_ru = {'LIMIT_SPEED': 'снижение скорости', 'REROUTE': 'новый маршрут', 'HOLD': 'удержание', 'SAFE_STOP': 'на площадку'}
    status_ru = {'DEGRADED': 'деградация', 'FAILED': 'отказ', 'UNKNOWN': 'нет данных'}
    src_ru = {'ROAD_DETECTOR': 'детектор', 'VIDEO_ANALYTICS': 'видеоаналитика', 'RSU': 'модуль V2X', 'WEATHER': 'метеостанция',
              'DIGITAL_TWIN': 'цифровой двойник'}

    def reason(h, v):
        o, a = h['odd'][v], h['actions'][v]
        if o['violation_codes']:
            return 'ODD: ' + ', '.join(odd_ru.get(x, x) for x in o['violation_codes'])
        code = (a['rationale_codes'] or [''])[0]
        text = code_text.get(code, code)
        return text[:1].lower() + text[1:].rstrip('.')

    def auto_chapters():
        found = []

        def add(step, title, short, text, select=None, focus=(), segments=(), prio=1):
            found.append({'step': step, 'title': title, 'short': short, 'text': text, 'select': select or veh_ids[0],
                          'focus': list(focus), 'segments': list(segments), 'prio': prio})
        h = history[min(len(history) - 1, 11)]
        cnt = Counter(a['motion_action'] for a in h['actions'].values())
        ok_src = sum(x['status'] == 'OK' for x in h['sources'].values())
        open_n = sum(x['state'] == 'OPEN' for x in h['state'].values())
        add(h['step'], 'Начало сценария', 'Начало',
            f'{cars(len(veh_ids))} на сети из {len(seg_ids)} направлений: открыто {open_n}, источников в норме {ok_src} из {len(src_ids)}. '
            f'Без ограничений едут {cnt["CONTINUE"]}.')
        for e in events:
            if e['kind'] == 'incident' and e['detected']:
                hit = step_of[e['detected']]
                target = e['label'].split(' · ')[-1]
                add(e['detected'], e['label'], e['label'].split(' · ')[0].split(' ')[0],
                    f'По разметке событие началось в {hhmmss(t0 + (e["onset"] - 1) * dt)}; контроллер отреагировал в {hit["time"]} '
                    f'(через {e["delay"]} с).', segments=[target] if target in c.segments else [], prio=3)
        titles = {'LIMIT_SPEED': ('Первое ограничение скорости', 'Скорость'), 'REROUTE': ('Первый новый маршрут', 'Маршрут'),
                  'HOLD': ('Первое удержание', 'Удержание'), 'SAFE_STOP': ('Первая безопасная остановка', 'Площадка')}
        for act, (title, short) in titles.items():
            for h in history:
                vs = [v for v in veh_ids if h['actions'][v]['motion_action'] == act]
                if vs:
                    v = vs[0]
                    add(h['step'], title, short, f'{v} на {h["tele"][v].get("segment_id", "—")}: {act_ru[act]} — {reason(h, v)}. '
                        f'С таким решением сейчас {cars(len(vs))}.', v, [v], prio=2)
                    break
        for h in history:
            vs = [v for v in veh_ids if h['actions'][v]['remote_support_required']]
            if vs:
                add(h['step'], 'Первая дистанционная поддержка', 'Оператор',
                    f'{vs[0]}: подключён оператор — {reason(h, vs[0])}. Сессий занято: {len(vs)} из {c.remote_limit}.', vs[0], [vs[0]])
                break
        for h in history:
            bad = [s for s, x in h['sources'].items() if x['status'] != 'OK']
            if bad:
                x = h['sources'][bad[0]]
                add(h['step'], f'Сбой источника {bad[0]}', 'Источник',
                    f'{bad[0]} ({src_ru.get(c.sources[bad[0]]["source_type"], "источник")}): '
                    f'{status_ru.get(x["status"], x["status"])}'
                    + (f' — {", ".join(fault_ru.get(f, f) for f in x["fault_types"])}' if x['fault_types'] else '')
                    + f'. Доверие снижено до {num(x["trust_score"], 2)}; '
                    f'источников с отклонениями: {len(bad)} из {len(src_ids)}.')
                break
        busy = [(sum(a['motion_action'] != 'CONTINUE' for a in h['actions'].values()), h) for h in history]
        top, peak = max(busy, key=lambda x: (x[0], -x[1]['step']))
        if top:
            pc = Counter(a['motion_action'] for a in peak['actions'].values() if a['motion_action'] != 'CONTINUE')
            add(peak['step'], 'Пик вмешательств', 'Пик',
                f'Больше всего машин с ограничениями: {top} из {len(veh_ids)} ('
                + ', '.join(f'{kind_ru.get(a, a)} — {n}' for a, n in pc.most_common()) + ').')
        last = history[-1]
        lc = Counter(a['motion_action'] for a in last['actions'].values())
        add(last['step'], 'Конец сценария', 'Итог',
            f'В конце: без ограничений {lc["CONTINUE"]}' + ''.join(f', {kind_ru.get(a, a)} — {n}' for a, n in lc.items() if a != 'CONTINUE')
            + (f'. Машин вне ODD без защитного действия (3 шага подряд): {unsafe_episodes}.' if has_labels else '.'))
        found.sort(key=lambda x: x['step'])
        kept = []
        for ch in found:  # moments closer than 30 s merge, the more important one stays; the final moment always stays
            if ch is not found[-1] and found[-1]['step'] - ch['step'] < 6:
                continue
            if kept and ch['step'] - kept[-1]['step'] < 6 and ch is not found[-1]:
                if ch['prio'] > kept[-1]['prio']:
                    kept[-1] = ch
                continue
            kept.append(ch)
        return kept[:9] + ([kept[-1]] if len(kept) > 10 else kept[9:10])

    # ---- chapters: texts are filled from data and asserted -----------------------------
    def acts(step, vids=None):
        h = step_of[step]
        return Counter(a['motion_action'] for v, a in h['actions'].items() if vids is None or v in vids)

    by_profile = defaultdict(list)
    for v, r in c.vehicles.items():
        by_profile[r['odd_profile_id']].append(v)
    chapters = []

    chapters = auto_chapters()
    for ch in chapters:
        ch['index'] = ch['step'] - 1

    # ---- totals for the verification card ---------------------------------------------
    checks = S['check']
    totals = None if not has_labels else {
        'roads': round(100 * sum(x[0] for x in checks) / (len(seg_ids) * n), 2),
        'sources': round(100 * sum(x[1] for x in checks) / (len(src_ids) * n), 2),
        'odd': round(100 * sum(x[2] for x in checks) / max(1, sum(x[3] for x in checks)), 2),
        'unsafeEpisodes': unsafe_episodes,
    }

    solution_sha = hashlib.sha256((ROOT / 'solution' / 'solution.py').read_bytes()).hexdigest()
    out = {
        'meta': {'scenario': SCENARIO, 'title': scenario_info.get('title', ''), 'summary': scenario_info.get('summary', ''),
                 'steps': n, 'dt': dt, 't0': hhmmss(t0), 'hasLabels': has_labels, 'remoteLimit': c.remote_limit,
                 'solution_sha256': solution_sha, 'stopReachSec': STOP_REACH_SEC},
        'nodes': [{'id': k, 'x': float(v['x_m']), 'y': float(v['y_m']), 'name': v['name']} for k, v in nodes.items()],
        'segments': [{'id': k, 'from': v['from_node'], 'to': v['to_node'], 'lanes': int(v['lanes']), 'group': v['road_group'],
                      'structure': v['structure'], 'weight': float(v['weight_limit_t']), 'speed': float(v['speed_limit_kmh']),
                      'zone': v['v2x_zone'], 'length': float(v['length_m']), 'av': int(v['av_allowed'])}
                     for k, v in c.segments.items()],
        'stops': [{'id': k, 'segment': v['segment_id'], 'capacity': int(v['capacity_vehicles']),
                   'mass': float(v['max_vehicle_mass_t'])} for k, v in c.stops.items()],
        'hubs': [{'id': k, 'node': v['node_id'], 'name': v['name'], 'capacity': int(v['capacity_vehicles'])} for k, v in c.hubs.items()],
        'sources': [{'id': k, 'type': v['source_type'], 'coverage': v['coverage']} for k, v in c.sources.items()],
        'zones': zones,
        'stations': [{'id': k, 'node': v['node_id']} for k, v in weather_stations.items()],
        'codeText': code_text,
        'profiles': c.profiles,
        'vehicles': [{'id': k, 'profile': v['odd_profile_id'], 'mass': float(v['gross_mass_t']), 'priority': int(v['cargo_priority']),
                      'origin': v['origin_hub_id'], 'cargo': orders.get(k, {}).get('cargo_class', '')} for k, v in c.vehicles.items()],
        'faultNames': FAULTS,
        'decisions': decision_table,
        'runs': runs,
        'routes': route_table,
        'steps': dict(S),
        'events': events,
        'chapters': chapters,
        'totals': totals,
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open('w', encoding='utf-8', newline='\n') as stream:
        stream.write('window.CORRIDOR_DEMO = ' + json.dumps(out, ensure_ascii=False, separators=(',', ':')) + ';\n')
    print(json.dumps({'steps': n, 'bytes': OUTPUT.stat().st_size, 'decisions': len(decision_table),
                      'chapters': [ch['step'] for ch in chapters], 'totals': totals}, ensure_ascii=False))


if __name__ == '__main__':
    main()
