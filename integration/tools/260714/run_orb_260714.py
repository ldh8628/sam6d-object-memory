#!/usr/bin/env python3
"""ORB-SLAM3 over the same 12 bags as run_rtab_260714.py, so both algorithms are
produced by the same harness on the same inputs.

The 260714 ORB-SLAM3 outputs already in orbslam_ws/output/260714 came from an
earlier session and two of them are unusable (sam_110532 has 1049 poses for 758
frames with 84 backward time jumps, i.e. several map resets concatenated), so
they are re-run here rather than trusted.  Run inside `conda activate orbslam3`.
"""
import importlib.util, json, os, sys
from pathlib import Path

ROOT = Path("/home/ldh9501/temp_ws/CLI_environment")
HERE = ROOT / "integration" / "tools" / "260714"
OUT = ROOT / "output_slam" / "260714_pairs"

spec = importlib.util.spec_from_file_location(
    "run_slam_0724", ROOT / "data_slam" / "0724_chungbuk" / "run_slam_0724.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
mod.SC_OUTPUT = OUT
mod.CONFIG_DIR = HERE / "configs_orb"
mod.ORB_BAG_RATE = float(os.environ.get("ORB_BAG_RATE", "1.0"))

sessions = json.loads((HERE / "manifest_260714_pairs.json").read_text())
only = sys.argv[1:]
if only:
    sessions = [s for s in sessions if s["name"] in only]
results = []
for i, ds in enumerate(sessions, 1):
    print(f"=== [{i}/{len(sessions)}] orbslam3 {ds['name']} ===", flush=True)
    r = mod.run_orb(ds)
    print("    " + json.dumps(r), flush=True)
    results.append(r)
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "run_summary_orbslam3.json").write_text(json.dumps(results, indent=1))
print("\n=== summary ===")
for r in results:
    ok = r.get("traj_rows", 0) > 1
    print(f"- [{'PASS' if ok else 'FAIL'}] {r['name']}: rows={r.get('traj_rows')} "
          f"kf={r.get('kf_rows')} {r.get('elapsed_sec')}s")
