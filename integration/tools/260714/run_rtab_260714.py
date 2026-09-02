#!/usr/bin/env python3
"""RTAB-Map over the 12 bags of the 260714 pairs, reusing run_slam_0724.run_rtab.

Stamps of colour/depth/camera_info are identical in these converted bags (checked),
so exact sync + RELIABLE QoS is used — the setting that fixed 260804 (odom updates
279 -> 775). Run inside `conda activate rtabmap`.
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
mod.RTAB_SYNC_ARGS = "approx_sync:=false"
mod.RTAB_QOS_ARGS = "qos:=1 qos_image:=1 qos_camera_info:=1"
mod.RTAB_BAG_RATE = os.environ.get("RTAB_BAG_RATE", "1.0")

sessions = json.loads((HERE / "manifest_260714_pairs.json").read_text())
only = sys.argv[1:]
if only:
    sessions = [s for s in sessions if s["name"] in only]
results = []
for i, ds in enumerate(sessions, 1):
    print(f"=== [{i}/{len(sessions)}] rtabmap {ds['name']} ===", flush=True)
    r = mod.run_rtab(ds)
    print("    " + json.dumps(r), flush=True)
    results.append(r)
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "run_summary_rtabmap.json").write_text(json.dumps(results, indent=1))
print("\n=== summary ===")
for r in results:
    ok = r["traj_rows"] > 1
    print(f"- [{'PASS' if ok else 'FAIL'}] {r['name']}: rows={r['traj_rows']} "
          f"nodes={r['nodes']} pts={r['dense_points']} {r['elapsed_sec']}s")
