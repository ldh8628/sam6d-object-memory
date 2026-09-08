#!/usr/bin/env python3
"""Camera-free failure-path tests of the real launcher/native node."""
import argparse,csv,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--atlas',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    settings=ROOT/'orb3_publish/config/TUM1.yaml';base=Path('/opt/orb-live/benchmarks')
    cases=[('counter_gap',['--realsense-bag',str(base/'software_xyz_gap.bag')],3,'counter gap'),
           ('zero_counter_gap',['--realsense-bag',str(base/'software_xyz_zero_gap.bag')],3,'counter gap'),
           ('fifo_overflow',['--dataset',str(base/'rgbd_dataset_freiburg1_rpy'),'--replay-rate','0','--queue-capacity','1'],3,'FIFO capacity'),
           ('no_camera',[],3,'no RealSense camera')]
    results=[]
    for name,extra,expected,reason in cases:
        out=a.output/name
        command=[sys.executable,str(ROOT/'run_orb3_publish.py'),'--localhost-only','--atlas',str(a.atlas),'--settings',str(settings),'--output',str(out),*extra]
        with (a.output/f'{name}.log').open('w') as log:
            run=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=90)
        result=json.loads((out/'summary.json').read_text())
        assert run.returncode==expected,(name,run.returncode,result)
        assert reason in result.get('error',''),(name,result)
        if name=='zero_counter_gap':
            assert max(result['rgb_gaps'],result['depth_gaps'])==2,result
            with (out/'frames.csv').open() as frames:
                first=next(csv.DictReader(frames))
            assert first['rgb_number']==first['depth_number']=='0',first
        assert not result['complete_input_processing'] and not result['full_requirement_verified']
        assert result['atlas_unchanged'],name
        results.append(dict(name=name,**result));print(name,'PASS',flush=True)
    (a.output/'faults.json').write_text(json.dumps(results,indent=2)+'\n')
if __name__=='__main__':main()
