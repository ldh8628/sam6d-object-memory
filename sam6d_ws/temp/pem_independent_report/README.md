# PEM independent verification smoke

Open `index.html` directly in a browser.

- frames: 2
- detections: 6
- dataset: longcircle2_sam
- camera source: Dedicated SAM D435i color camera; aligned conversion of Windows SDK recording
- partial mode: False
- GT: No ground truth: SAM-to-SLAM extrinsic is unavailable (extrinsic_missing); O/X is unavailable.

Regenerate with:

```bash
python tools/build_pem_explorer.py --bag data/longcircle2_sam --detections temp/pem_independent_parity/dump.jsonl --frames temp/pem_independent_parity/dump_frames.jsonl --out temp/pem_independent_report --title 'PEM independent verification smoke' --gt-unavailable-reason auto --render-topn 3
```
