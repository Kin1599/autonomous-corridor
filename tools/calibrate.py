"""Fit weather source coverage from TRAIN only. No time/scenario lookup in runtime."""
import argparse,csv,gzip,json,pathlib,collections
import numpy as np

def fit(data,scenarios,out):
    segs=list(csv.DictReader(open(data/'01_reference/02_network_segments.csv')));sids=[r['segment_id'] for r in segs]
    total=collections.defaultdict(lambda:np.zeros(len(sids)));n=collections.Counter()
    for sc in scenarios:
        folder=data/'02_train'/sc;truth={}
        with gzip.open(folder/'labels/01_segment_state.csv.gz','rt') as f:
            for r in csv.DictReader(f):truth.setdefault(r['timestamp'],{})[r['segment_id']]=(float(r['visibility_m']),float(r['rain_level']))
        arrays={t:np.array([rows[s] for s in sids]) for t,rows in truth.items()}
        with gzip.open(folder/'packets.ndjson.gz','rt') as f:
            for line in f:
                for e in json.loads(line)['events']:
                    if e['event_type']!='WEATHER_OBSERVATION' or e['event_time'] not in arrays:continue
                    arr=arrays[e['event_time']];err=((arr[:,0]-e['visibility_m'])/100)**2+4*(arr[:,1]-e['rain_level'])**2
                    total[e['source_id']]+=np.minimum(err,100);n[e['source_id']]+=1
    mapping={sid:segs[int(np.argmin(err/n[sid]))]['road_group'] for sid,err in total.items()}
    out.mkdir(parents=True,exist_ok=True);(out/'weather_groups.json').write_text(json.dumps(mapping,indent=2));return mapping
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);p.add_argument('--folds',action='store_true');a=p.parse_args();sc=[p.name for p in sorted((a.data/'02_train').glob('*'))]
    print('all',fit(a.data,sc,a.out))
    if a.folds:
        for held in sc:print(held,fit(a.data,[s for s in sc if s!=held],a.out/'folds'/held))
