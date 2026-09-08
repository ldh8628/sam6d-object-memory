#!/usr/bin/env python3
"""Repeat replay images with strictly increasing 30Hz timestamps for a load soak.
This is a synthetic timeline, NOT an uninterrupted physical camera recording.
"""
import argparse,json
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--frames',type=int,default=18000);p.add_argument('--fps',type=float,default=30)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    rows=[line.split() for line in (a.source/'associations.txt').read_text().splitlines() if line and not line.startswith('#')]
    (a.output/'rgb').symlink_to(a.source.resolve()/'rgb',target_is_directory=True)
    (a.output/'depth').symlink_to(a.source.resolve()/'depth',target_is_directory=True)
    with (a.output/'associations.txt').open('w') as f:
        for i in range(a.frames):
            row=rows[i%len(rows)];stamp=float(rows[0][0])+i/a.fps
            f.write(f'{stamp:.9f} {row[1]} {stamp:.9f} {row[3]}\n')
    (a.output/'source.json').write_text(json.dumps(dict(source=str(a.source),frames=a.frames,fps=a.fps,
                                                    synthetic_timeline=True,repeated_images=True),indent=2)+'\n')
if __name__=='__main__':main()
