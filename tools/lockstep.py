"""Send exactly one packet, wait for one response, then make next packet available."""
import argparse,gzip,hashlib,json,pathlib,select,subprocess,sys,time

def run(data,scenario,output,timeout=2,python=sys.executable):
    folder=next(data.glob('*/'+scenario));here=pathlib.Path(__file__).resolve().parent.parent/'solution'
    import jsonschema
    validator=jsonschema.Draft202012Validator(json.loads((data/'04_contract/03_decision.schema.json').read_text()),format_checker=jsonschema.FormatChecker())
    log=output.with_suffix(output.suffix+'.stderr.log');lat=[];actions={};remote_peak=0;count=0
    with log.open('w') as err:
        child=subprocess.Popen([python,'-u',str(here/'solution.py'),'--reference',str(data/'01_reference')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err)
        try:
            with gzip.open(folder/'packets.ndjson.gz','rt') as src,gzip.GzipFile(filename=str(output),mode='wb',mtime=0) as dst:
                for line in src:
                    start=time.perf_counter();child.stdin.write(line.encode());child.stdin.flush()
                    if not select.select([child.stdout],[],[],timeout if count else 60)[0]:raise TimeoutError('No response before deadline')
                    response=child.stdout.readline();elapsed=(time.perf_counter()-start)*1000
                    if not response:raise RuntimeError('Unexpected EOF from controller')
                    if count and elapsed>timeout*1000:raise TimeoutError('Response exceeded deadline')
                    p=json.loads(line);d=json.loads(response);validator.validate(d)
                    assert d['packet_id']==p['packet_id'] and d['scenario_id']==p['scenario_id'] and d['decision_time']==p['decision_time']
                    for field,key in [('state_estimates','segment_id'),('source_assessments','source_id'),('vehicle_assessments','vehicle_id'),('vehicle_actions','vehicle_id')]:
                        ids=[r[key] for r in d[field]];assert len(ids)==len(set(ids))
                    remote_peak=max(remote_peak,sum(a['remote_support_required'] for a in d['vehicle_actions']));assert remote_peak<=6
                    for a in d['vehicle_actions']:actions[a['motion_action']]=actions.get(a['motion_action'],0)+1
                    dst.write(response);lat.append(elapsed);count+=1
            child.stdin.close();assert child.wait(timeout=10)==0;assert child.stdout.read()==b''
        finally:
            if child.poll() is None:child.kill();child.wait()
    lat.sort();report={'scenario':scenario,'packets':count,'schema_valid':count,'remote_peak':remote_peak,'actions':actions,'latency_ms':{'mean':sum(lat)/len(lat),'p99':lat[min(len(lat)-1,int(len(lat)*.99))],'max':max(lat)},'sha256_uncompressed':hashlib.sha256(gzip.open(output,'rb').read()).hexdigest(),'execution':'native Python, causal lockstep'}
    output.with_suffix(output.suffix+'.report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data',type=pathlib.Path,required=True);p.add_argument('--scenario',required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--python',default=sys.executable);a=p.parse_args();run(a.data,a.scenario,a.output,python=a.python)
