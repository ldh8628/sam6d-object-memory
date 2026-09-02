#!/usr/bin/env python3
"""3단계-B — RTAB-Map (RGB-D). 궤적과 2D/3D 맵을 만든다.

    python integration/pipeline/rtabmap.py 185223__circle_ccw_8laps

산출물: <출력>/rtabmap/{trajectory.txt, map_2d.pgm, map_2d.yaml, map_2d.png, map_3d.ply}

두 가지가 반드시 필요하다:
  * exact sync + RELIABLE QoS — 변환 bag 은 color/depth 스탬프가 같으므로
    approx_sync 는 오히려 해롭다 (0724 에서 odom 279 -> 775 로 갈렸다).
  * 조밀화 — RTAB 이 저장하는 것은 그래프 노드(~1 Hz, 최대 18 s 공백)라 그대로는
    객체 융합의 시간 허용치(0.2 s)에 대부분 걸리지 않는다. 노드가 적용한 보정량을
    프레임레이트 odometry 에 입혀 되살린다.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402
import gridmap as GM  # noqa: E402

RUN_SLAM = C.TOOLS_ROOT / "run_slam_0807.py"
DENSIFY = C.TOOLS_ROOT / "densify_0807.py"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="bag 재생 배속. rate 1.0 에서 graph_poses 286 으로 "
                         "0.5 와 동일함을 실측했다(시간은 594s -> 298s).")
    ap.add_argument("--res", type=float, default=0.05, help="격자 해상도 [m/cell]")
    ap.add_argument("--frame-stride", type=int, default=6,
                    help="격자에 쓸 depth 프레임 간격 (6 = 5 Hz)")
    ap.add_argument("--min-hits", type=int, default=2,
                    help="셀을 장애물로 보려면 필요한 최소 적중 수")
    ap.add_argument("--domain", type=int, default=91, help="ROS_DOMAIN_ID (토픽 충돌 격리)")
    ap.add_argument("--no-ply", dest="ply", action="store_false",
                    help="3D map_3d.ply 를 만들지 않는다")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="3D 점군 복셀 크기 [m]")
    ap.add_argument("--force", action="store_true",
                    help="궤적이 이미 있어도 SLAM 을 다시 돌린다")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    out = C.out_dir(sess)
    stage = out / "_stage"
    pair = f"{C.DATASET}/{sess}/_stage"
    d = stage / "slam_rtabmap"

    dst = out / "rtabmap"
    dst.mkdir(parents=True, exist_ok=True)
    final = dst / "trajectory.txt"
    if a.force or not final.is_file():
        if a.force or not (d / "trajectory.txt").is_file():
            man = C.write_manifest(sess, stage)
            C.run(f"{C.CONDA} run -n rtabmap --no-capture-output python {RUN_SLAM} "
                  f"--method rtabmap --pair {pair} --session {sess} --manifest {man} "
                  f"--rate {a.rate} --domain {a.domain}",
                  log_path=out / "logs" / "rtabmap.log", label="RTAB-Map RGB-D")
            C.run(f"{C.CONDA} run -n hdl_graph_slam_humble --no-capture-output python "
                  f"{DENSIFY} --pair {pair} --method rtabmap",
                  log_path=out / "logs" / "rtabmap_densify.log", label="궤적 조밀화")
        if not (d / "trajectory.txt").is_file():
            C.die(f"RTAB-Map 이 궤적을 내지 못했다 — 로그: {out / 'logs'}")
        shutil.copyfile(d / "trajectory.txt", final)
    else:
        C.log(f"궤적이 이미 있어 SLAM 을 건너뛴다 ({final}) — 다시 돌리려면 --force")

    # ORB3 와 완전히 같은 코드로, bag 의 depth + 이 궤적으로 만든다. 그래서 두 지도의
    # 차이는 '점군 생성 방식'이 아니라 **궤적 차이만** 반영한다.
    g, zup = GM.backend_grid("rtabmap", out, C.data_dir(sess), C.load_info(sess),
                             res=a.res, stride=a.frame_stride, min_hits=a.min_hits)
    GM.write_map(g, zup, dst / "map_2d", "RTAB-Map")
    if a.ply:
        # 3D 지도는 각 백엔드의 dense 점군이 아니라 **이 궤적으로 bag 을 재투영**해
        # 만든다. 궤적과 점군이 같은 실행에서 나오므로 유니티에서 겹쳐도 어긋나지 않고,
        # trajectory.txt 와 같은 프레임이라 좌표 변환이 한 번으로 끝난다.
        r = GM.backend_ply("rtabmap", out, C.data_dir(sess), C.load_info(sess),
                           voxel=a.voxel)
        C.log(f"  3D 맵 {r['points']:,} pts · rgb={r['rgb']} · {r['size_mb']} MB "
              f"-> {dst / 'map_3d.ply'}")
    C.log(f"완료 — {dst} ({len(zup)} poses)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
