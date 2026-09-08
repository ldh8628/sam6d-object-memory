#!/usr/bin/env python3
"""build_hsv_template_cache.py — precompute per-object HSV reference caches (Phase 1B).

For every enabled object in the config, builds the 42-view render HSV histogram bank
(ism_hsv.build_reference) and writes `<feature_dir>/<name>_hsv.npz` with a content
signature + authority version for change detection. This is an OFFLINE tool — the
runtime NEVER builds caches (spec: no generation at operation start).

Usage:
  ~/miniconda3/envs/sam_yolo/bin/python tools/build_hsv_template_cache.py [--config PATH] [--force]
"""
import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import ism_hsv                       # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    ap.add_argument("--force", action="store_true", help="rebuild even if a valid cache exists")
    a = ap.parse_args()

    defaults, objs = o_n.load_config(a.config)
    print(f"authority = {ism_hsv.HSV_AUTHORITY_VERSION}")
    print(f"params: hue_shift={ism_hsv.HUE_SHIFT_DEG} sat_gain={ism_hsv.SAT_GAIN} "
          f"bins={ism_hsv.HBINS}x{ism_hsv.SBINS} sub={ism_hsv.SUB_N}\n")
    n_ok = 0
    for o in objs:
        tdir = o["template_dir"]
        fdir = os.path.dirname(o["cls_cache"])
        path = o.get("hsv_cache") or os.path.join(fdir, f"{o['name']}_hsv.npz")
        hc = int(o.get("hsv_hue_correction_ocv", 0))     # Phase 1C class color correction
        if not a.force:
            proto, meta = ism_hsv.load_cache(path, tdir)
            if proto is not None and int(meta.get("hue_correction_ocv", 0)) == hc:
                print(f"  [skip valid] {o['name']:22s} views={proto.shape[0]} hc={hc} -> {path}")
                n_ok += 1
                continue
        proto, nv = ism_hsv.build_reference(tdir, hue_correction_ocv=hc)
        if nv == 0:
            print(f"  [WARN empty] {o['name']:22s} no usable render views in {tdir}")
            continue
        ism_hsv.save_cache(path, proto, tdir, nv, hue_correction_ocv=hc)
        tag = f" hue_correction={hc}ocv" if hc else ""
        print(f"  [built] {o['name']:22s} views={nv} shape={proto.shape}{tag} -> {path}")
        n_ok += 1
    print(f"\n{n_ok}/{len(objs)} objects have a valid HSV cache.")


if __name__ == "__main__":
    main()
