#!/usr/bin/env python3
"""Associate each RGB/depth image at most once, report unmatched input images."""
import argparse
import json
from pathlib import Path


def associate(rgb, depth, max_difference=0.02):
    # Global greedy association by absolute timestamp difference, as in TUM's
    # reference associate.py. A depth frame is never reused for another RGB.
    candidates=sorted((abs(a-b),a,b) for a in rgb for b in depth if abs(a-b)<max_difference)
    available_rgb=set(rgb); available_depth=set(depth); matches=[]
    for _,a,b in candidates:
        if a in available_rgb and b in available_depth:
            available_rgb.remove(a); available_depth.remove(b); matches.append((a,b))
    return sorted(matches),sorted(available_rgb),sorted(available_depth)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory',type=Path)
    ap.add_argument('--max-difference',type=float,default=0.02)
    args=ap.parse_args()
    def read(name):
        return {float(parts[0]):parts[1] for line in (args.directory/name).read_text().splitlines()
                if (parts:=line.split()) and not parts[0].startswith('#')}
    rgb=read('rgb.txt'); depth=read('depth.txt')
    matches,missing_rgb,missing_depth=associate(rgb,depth,args.max_difference)
    (args.directory/'associations.txt').write_text(''.join(f'{a:.6f} {rgb[a]} {b:.6f} {depth[b]}\n' for a,b in matches))
    report=dict(rgb_frames=len(rgb),depth_frames=len(depth),associated_pairs=len(matches),
                unmatched_rgb=len(missing_rgb),unmatched_depth=len(missing_depth),
                max_difference=args.max_difference,unmatched_rgb_timestamps=missing_rgb,unmatched_depth_timestamps=missing_depth)
    (args.directory/'association_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if not k.endswith('timestamps')}))

if __name__=='__main__': main()
