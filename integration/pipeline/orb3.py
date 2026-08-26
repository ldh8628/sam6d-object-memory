#!/usr/bin/env python3
"""3단계-A — ORB-SLAM3 (RGB-D). 궤적과 2D/3D 맵을 만든다.

    python integration/pipeline/orb3.py 185223__circle_ccw_8laps

산출물: <출력>/orb3/{trajectory.txt, map_2d.pgm, map_2d.yaml, map_2d.png, map_3d.ply}

궤적의 map 프레임은 첫 카메라의 광학 프레임이다(y 아래). 2D 맵은 z-up 으로 돌려서
바닥 위 슬라이스를 격자화한 것이며, 세 백엔드에 완전히 같은 코드를 쓴다.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402
import gridmap as GM  # noqa: E402

RUN_INTEGRATION = C.INTEGRATION_DIR / "run_integration.py"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--rate", type=float, default=1.0, help="bag 재생 배속")
    ap.add_argument("--res", type=float, default=0.05, help="격자 해상도 [m/cell]")
    ap.add_argument("--frame-stride", type=int, default=6,
                    help="격자에 쓸 depth 프레임 간격 (6 = 5 Hz)")
    ap.add_argument("--min-hits", type=int, default=2,
                    help="셀을 장애물로 보려면 필요한 최소 적중 수")
    ap.add_argument("--no-ply", dest="ply", action="store_false",
                    help="3D map_3d.ply 를 만들지 않는다")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="3D 점군 복셀 크기 [m]")
    ap.add_argument("--force", action="store_true",
                    help="궤적이 이미 있어도 SLAM 을 다시 돌린다")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    data, out = C.data_dir(sess), C.out_dir(sess)
    if not (data / "SLAM" / "metadata.yaml").is_file():
        C.die(f"{data}/slam 이 없다 — 먼저 convert.py 를 돌려라")
    stage = out / "_stage"
    d = stage / "slam_orb3"

    dst = out / "orb3"
    dst.mkdir(parents=True, exist_ok=True)
    final = dst / "trajectory.txt"
    if a.force or not final.is_file():
        if a.force or not (d / "trajectory.txt").is_file():
            C.run(f"python3 {RUN_INTEGRATION} {data} --stage slam --out {stage} "
                  f"--slam-dir slam_orb3 --rate {a.rate}",
                  log_path=out / "logs" / "orb3.log", label="ORB-SLAM3 RGB-D")
        if not (d / "trajectory.txt").is_file():
            C.die(f"ORB-SLAM3 가 궤적을 내지 못했다 — 로그: {out / 'logs' / 'orb3.log'}")
        shutil.copyfile(d / "trajectory.txt", final)
    else:
        # 지도만 다시 만들 때 SLAM 5분을 다시 돌리지 않는다. 2D/3D 지도는 궤적과 bag 만
        # 있으면 만들 수 있으므로 _stage 가 지워진 뒤에도 재생성된다.
        C.log(f"궤적이 이미 있어 SLAM 을 건너뛴다 ({final}) — 다시 돌리려면 --force")

    # 2D 맵은 합쳐진 dense 점군이 아니라 **bag 의 depth + 이 궤적**으로 만든다.
    # 합친 점군에는 어느 프레임에서 봤는지가 없어 광선을 쏠 원점이 없기 때문이다.
    g, zup = GM.backend_grid("orb3", out, data, C.load_info(sess), res=a.res,
                             stride=a.frame_stride, min_hits=a.min_hits)
    GM.write_map(g, zup, dst / "map_2d", "ORB-SLAM3")
    if a.ply:
        # 3D 지도는 각 백엔드의 dense 점군이 아니라 **이 궤적으로 bag 을 재투영**해
        # 만든다. 궤적과 점군이 같은 실행에서 나오므로 유니티에서 겹쳐도 어긋나지 않고,
        # trajectory.txt 와 같은 프레임이라 좌표 변환이 한 번으로 끝난다.
        r = GM.backend_ply("orb3", out, C.data_dir(sess), C.load_info(sess),
                           voxel=a.voxel)
        C.log(f"  3D 맵 {r['points']:,} pts · rgb={r['rgb']} · {r['size_mb']} MB "
              f"-> {dst / 'map_3d.ply'}")
    C.log(f"완료 — {dst} ({len(zup)} poses)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
