#!/usr/bin/env python3
"""Read actual tabletop_servo JSON files offline. No SDK import or robot access.

Missing counters stay null, not zero. Nominal duration is not wall-clock duration.
Aggregate only reports with all required counters; never infer medians/p99.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

def obj(v): return v if isinstance(v,dict) else {}
def num(v): return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)

def summarize(path):
    raw=path.read_bytes();d=json.loads(raw)
    if not isinstance(d,dict):raise ValueError('JSON top level is not an object')
    f=obj(d.get('field_runtime_servo'));t=obj(d.get('timing'));x=obj(d.get('sdk_latency_probe'))
    n=x.get('pulse_calls');p=f.get('period_s',t.get('period_s'))
    mode=f.get('mode') or ('PLAIN_'+str(round(p*1000))+'MS_S'+str(f.get('stride')) if num(p) else 'UNKNOWN')
    r={'file':path.name,'sha256':hashlib.sha256(raw).hexdigest(),'status':d.get('status'),'mode':mode,
       'trajectory_sha256':d.get('trajectory_sha256'),'local_ip':d.get('local_ip'),'frames':f.get('runtime_motion_frames'),
       'period_s':p,'nominal_duration_s':f.get('runtime_duration_s'),'pulse_calls':n,
       'pulse_mean_ms':x.get('pulse_mean_ms'),'pulse_max_ms':x.get('pulse_max_ms'),
       'pulse_median_ms':x.get('pulse_median_ms'),'pulse_p99_ms':x.get('pulse_p99_ms'),
       'pulse_over_20ms':x.get('pulse_over_20ms'),'pulse_over_40ms':x.get('pulse_over_40ms'),
       'realigns':t.get('realigns'),'mean_late_ms':t.get('mean_late_ms'),'max_late_ms':t.get('max_late_ms'),
       'cleanup_status':obj(d.get('cleanup')).get('status'),
       'network_state':'UNKNOWN_UNLESS_SEPARATE_TIMESTAMPED_ROUTER_EVIDENCE_PROVIDED'}
    for k in ('pulse_over_20ms','pulse_over_40ms'):
        r[k+'_pct']=100*r[k]/n if num(n) and n>0 and num(r[k]) else None
    events=x.get('slow_events',[])
    if isinstance(events,list):
        ev=[e for e in events if isinstance(e,dict) and num(e.get('duration_ms'))]
        r['top5_slow_events']=sorted(ev,key=lambda e:e['duration_ms'],reverse=True)[:5]
    return r

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('files',nargs='*',type=Path)
    ap.add_argument('--root',type=Path,default=Path('.'))
    ap.add_argument('--out',type=Path,help='Write a NEW summary JSON; refuses overwrite')
    a=ap.parse_args();paths=a.files or sorted(a.root.glob('tabletop_servo_*.json'))
    if not paths:ap.error('No report files found')
    rows=[];errors=[]
    for p in paths:
        try:rows.append(summarize(p))
        except (OSError,ValueError,TypeError) as e:errors.append({'file':str(p),'error':str(e)})
    groups={}
    for r in rows:
        if r['status']!='SERVO_COMPLETED_RETURNED_HOME':continue
        k=(r['mode'],r['trajectory_sha256'],r['period_s'])
        groups.setdefault(k,[]).append(r)
    agg=[]
    for (mode,sha,period),rs in groups.items():
        complete=[r for r in rs if num(r['pulse_calls']) and r['pulse_calls']>0 and all(num(r[k])for k in ('pulse_mean_ms','pulse_max_ms','pulse_over_20ms','pulse_over_40ms','realigns'))]
        n=sum(r['pulse_calls']for r in complete)
        row={'mode':mode,'trajectory_sha256':sha,'period_s':period,'reports':len(rs),'complete_counter_reports':len(complete),'excluded_incomplete_reports':len(rs)-len(complete)}
        if n:
            row.update(pulse_calls=n,weighted_mean_ms=sum(r['pulse_mean_ms']*r['pulse_calls']for r in complete)/n,
                       max_ms=max(r['pulse_max_ms']for r in complete),over20=sum(r['pulse_over_20ms']for r in complete),
                       over40=sum(r['pulse_over_40ms']for r in complete),realigns=sum(r['realigns']for r in complete))
            row.update(over20_pct=100*row['over20']/n,over40_pct=100*row['over40']/n)
        agg.append(row)
    result={'schema':'servo_offline_report_summary.v1','artifact_class':'DERIVED_SUMMARY_NOT_ORIGINAL_REPORTS',
            'runs':rows,'groups':agg,'errors':errors,'limitations':['No full-sample median or p99 is reconstructed','No network ON/OFF inferred from local IP','No wall-clock duration inferred from nominal duration','Aggregate grouping does not establish controlled experimental conditions']}
    text=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if a.out:
        with a.out.open('x',encoding='utf-8') as f:f.write(text)
        print(a.out)
    else:print(text,end='')
    return 1 if errors else 0
if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError) as exc:
        print('ERROR:',exc,file=sys.stderr);raise SystemExit(1)
