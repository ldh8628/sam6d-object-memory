#!/usr/bin/env python3
"""1단계 — SDK 원본을 표준 bag 으로. **converting_launch/convert_all.py 로 옮겨졌다.**

2026-08-25 재구성 이후 변환 진입점은 레포에 하나뿐이다:

    python3 converting_launch/convert_all.py \
        --inputs:=data_slam/<날짜>/<세션> --outputs:=data_slam_converted/<날짜>/<세션>

이 파일은 옛 호출부(`python integration/pipeline/convert.py <세션>`)를 살려 두기 위한
얇은 껍데기다. 새 스크립트를 그대로 부른다.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402

CONVERT_ALL = C.ROOT / "converting_launch" / "convert_all.py"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="data_slam/<날짜>/<세션> 또는 세션 이름")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--stages", default="clock,convert,lidar,imu",
                   help="extrinsic 까지 하려면 clock,convert,lidar,imu,extrinsic")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    cmd = [sys.executable, str(CONVERT_ALL),
           f"--inputs:={C.RAW_ROOT / sess}",
           f"--outputs:={C.DATA_ROOT / sess}",
           f"--stages:={a.stages}", f"--stride:={a.stride}"]
    if a.force:
        cmd.append("--force")
    C.log("converting_launch/convert_all.py 로 넘긴다 — " + " ".join(cmd[2:]))
    return subprocess.run(cmd, cwd=str(C.ROOT)).returncode


if __name__ == "__main__":
    raise SystemExit(main())
