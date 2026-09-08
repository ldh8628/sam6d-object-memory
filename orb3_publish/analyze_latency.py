#!/usr/bin/env python3
import argparse,csv,json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--cycle',type=int,default=694);a=p.parse_args()
    with (a.run/'frames.csv').open() as f:rows=list(csv.DictReader(f))
    blocks=[]
    for start in range(0,len(rows),2000):
        part=rows[start:start+2000]
        blocks.append(dict(first=start+1,last=start+len(part),valid=sum(r['tracking']=='1' for r in part),
          **{key:np.quantile([float(r[key]) for r in part],[.5,.95,.99]).tolist() for key in ('queue_ms','track_ms','enqueue_to_publish_ms')}))
    phases=[]
    for phase in range(a.cycle):
        part=rows[phase::a.cycle]
        if len(part)<2:continue
        phases.append(dict(phase=phase+1,track_median_ms=float(np.median([float(r['track_ms']) for r in part])),
                           late=sum(float(r['track_ms'])>33 for r in part),valid=sum(r['tracking']=='1' for r in part)))
    worst=sorted(rows,key=lambda r:float(r['track_ms']),reverse=True)[:20]
    cpus=[]
    if rows and 'track_cpu_ms' in rows[0]:
        for cpu in sorted(set(r['cpu_start'] for r in rows)):
            part=[r for r in rows if r['cpu_start']==cpu]
            cpus.append(dict(cpu=cpu,frames=len(part),migrations=sum(r['cpu_start']!=r['cpu_end'] for r in part),
                track_ms=np.quantile([float(r['track_ms']) for r in part],[.5,.95,.99]).tolist(),
                tracking_thread_cpu_ms=np.quantile([float(r['track_cpu_ms']) for r in part],[.5,.95,.99]).tolist()))
    report=dict(blocks=blocks,cpu_groups=cpus,slow_phases=sorted(phases,key=lambda p:p['track_median_ms'],reverse=True)[:20],
                worst=[{k:r[k] for k in ('sequence','tracking','map_matches','queue_ms','track_ms','enqueue_to_publish_ms')} for r in worst])
    if rows and 'pair_wait_ms' in rows[0]:
        stage_keys=('pair_wait_ms','align_copy_ms','tracking_fifo_ms','track_ms','enqueue_to_publish_ms')
        report['first_12_frames']=[{k:r[k] for k in ('sequence',*stage_keys)} for r in rows[:12]]
        cutoff=int(rows[0]['enqueue_ns'])+10**9
        for label,part in [('initial_one_second',[r for r in rows if int(r['enqueue_ns'])<cutoff]),
                           ('after_initial_one_second',[r for r in rows if int(r['enqueue_ns'])>=cutoff])]:
            if part:
                report[label]=dict(frames=len(part),percentiles='p50,p99,max; diagnostic grouping only, not acceptance',
                    **{k:np.quantile([float(r[k]) for r in part],[.5,.99,1]).tolist() for k in stage_keys})
    (a.run/'latency_analysis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
