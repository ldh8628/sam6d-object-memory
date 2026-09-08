# PEM explorer diagnostic smoke

Open `index.html` directly in a browser.

- frames: 50
- detections: 1
- partial mode: False
- GT: Exploratory SLAM pseudo-GT, not externally measured ground truth.

Regenerate with:

```bash
python tools/build_pem_explorer.py --bag data/longcircle2 --detections temp/pem_explorer_diag_smoke/dump.jsonl --frames temp/pem_explorer_diag_smoke/dump_frames.jsonl --pseudo-gt output/longcircle2_experiments/20260822_slam_eval_final/pseudo_ground_truth.json --trajectory output/longcircle2_experiments/20260822_slam/CameraTrajectory.txt --out temp/pem_explorer_diag_report --title PEM explorer diagnostic smoke --render-topn 2
```
