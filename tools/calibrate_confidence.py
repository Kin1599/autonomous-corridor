"""Empirical confidence from out-of-fold prediction confusion counts."""
import argparse,json,pathlib

def fit(folder,output,exclude=None):
    result={};mapping={'segment':'state_estimates','source':'source_assessments','odd':'vehicle_assessments'}
    for key,outkey in mapping.items():
        count={}
        for file in sorted(folder.glob('*.metrics.json')):
            if exclude and file.name.startswith(exclude):continue
            m=json.loads(file.read_text())[key];names=m['confusion_labels'];matrix=m['confusion']
            for j,cls in enumerate(names):
                good,total=count.get(cls,(0,0));count[cls]=(good+matrix[j][j],total+sum(row[j] for row in matrix))
        result[outkey]={cls:round((good+9)/(total+10),6) for cls,(good,total) in count.items() if total>=20 and cls!='UNKNOWN'}
    output.write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folds',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--exclude');a=p.parse_args();print(fit(a.folds,a.output,a.exclude))
