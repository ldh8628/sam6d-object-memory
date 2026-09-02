#!/usr/bin/env python3
"""3단계-C — hdl_graph_slam (Velodyne LiDAR). 궤적과 2D/3D 맵을 만든다.

    python integration/pipeline/hdl.py 185223__circle_ccw_8laps

산출물: <출력>/hdl/{trajectory.txt, map_2d.pgm, map_2d.yaml, map_2d.png, map_3d.ply}

⚠ 이 궤적과 맵은 **LiDAR(Velodyne) 프레임**이다. ORB3/RTAB 의 카메라 map 프레임과
   다르다. 카메라 검출과 융합하려면 T_velo_cam 이 필요한데, 이는 캘리브 값이 아니라
   ORB3 궤적과의 hand-eye 정합으로 추정해야 하므로 fuse_video.py 가 담당한다.
   즉 hdl.py 자체는 ORB3 에 의존하지 않지만, 융합 결과는 부분적으로 의존한다.

⚠ 0724·260804 에서 원본 /velodyne_points 가 실제와 좌우 반전돼 있다는 것이 확인된 적이
   있다(IMU 중력·자이로 대조). 이 맵을 카메라 맵과 겹쳐 볼 때 좌우가 뒤집혀 보이면
   이 문제일 수 있다 — 궤적 정합은 거울을 흡수해 버려서 정합만으로는 검출되지 않는다.

hdl 이 저장하는 것은 키프레임(~0.7 Hz)이라 그대로는 융합의 시간 허용치에 걸리지 않는다.
스캔매칭 odometry(9.9 Hz)에 키프레임 보정량을 입혀 조밀화한다.
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--rate", type=float, default=0.5, help="bag 재생 배속")
    ap.add_argument("--res", type=float, default=0.05, help="격자 해상도 [m/cell]")
    ap.add_argument("--min-hits", type=int, default=2,
                    help="셀을 장애물로 보려면 필요한 최소 적중 수")
    ap.add_argument("--domain", type=int, default=80, help="ROS_DOMAIN_ID (토픽 충돌 격리)")
    ap.add_argument("--no-ply", dest="ply", action="store_false",
                    help="3D map_3d.ply 를 만들지 않는다")
    ap.add_argument("--voxel", type=float, default=0.03,
                    help="3D 점군 복셀 크기 [m]")
    ap.add_argument("--force", action="store_true",
                    help="궤적이 이미 있어도 SLAM 을 다시 돌린다")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    info = C.load_info(sess)
    if not info.get("lidar"):
        C.die("이 세션에는 Velodyne bag 이 없다 — hdl 은 돌릴 수 없다")
    out = C.out_dir(sess)
    stage = out / "_stage"
    pair = f"{C.DATASET}/{sess}/_stage"
    d = stage / "slam_hdl"

    dst = out / "hdl"
    dst.mkdir(parents=True, exist_ok=True)
    final = dst / "trajectory.txt"
    if a.force or not final.is_file():
        if a.force or not (d / "trajectory_velo_dense.txt").is_file():
            man = C.write_manifest(sess, stage)
            C.run(f"{C.CONDA} run -n hdl_graph_slam_humble --no-capture-output python {RUN_SLAM} "
                  f"--method hdl --pair {pair} --session {sess} --manifest {man} "
                  f"--rate {a.rate} --domain {a.domain}",
                  log_path=out / "logs" / "hdl.log", label="hdl_graph_slam")
            # densify_hdl_0807.py 의 조밀화만 쓴다. 카메라 프레임 변환은 fuse_video.py 담당.
            C.run(f"{C.CONDA} run -n hdl_graph_slam_humble --no-capture-output python -c "
                  f"\"import sys; sys.path.insert(0, r'{C.TOOLS_ROOT}'); "
                  f"from pathlib import Path; import densify_hdl_0807 as D; "
                  f"raise SystemExit(0 if D.densify(Path(r'{d}')) else 1)\"",
                  log_path=out / "logs" / "hdl_densify.log", label="궤적 조밀화")
        if not (d / "trajectory_velo_dense.txt").is_file():
            C.die(f"hdl 이 궤적을 내지 못했다 — 로그: {out / 'logs'}")
        shutil.copyfile(d / "trajectory_velo_dense.txt", final)
    else:
        C.log(f"궤적이 이미 있어 SLAM 을 건너뛴다 ({final}) — 다시 돌리려면 --force")

    # 키프레임마다 그 키프레임의 센서 원점에서 광선을 쏜다. 합친 점군에 '가장 가까운
    # 포즈'를 갖다 붙이면 한 번만 찍힌 셀(사람·원거리 희소 반사)이 지워지지 않아
    # 점유 셀의 38.8% 가 그런 잡점이었다.
    g, zup = GM.backend_grid("hdl", out, C.data_dir(sess), info, res=a.res,
                             min_hits=a.min_hits)
    GM.write_map(g, zup, dst / "map_2d", "hdl_graph_slam (LiDAR frame)")
    if a.ply:
        # 3D 지도는 hdl 공식 맵 생성(map_cloud_generator)과 같은 입력을 쓴다: prefiltering
        # 을 통과한 키프레임 점군을 그래프가 최적화한 pose 로 옮긴 것(graph_dump). 이건
        # trajectory.txt 와 같은 hdl map 프레임이라 유니티에서 겹쳐도 어긋나지 않는다.
        # graph_dump 가 없으면 bag 재투영으로 물러서는데, 그때는 동적 객체가 걸러지지 않는다.
        r = GM.backend_ply("hdl", out, C.data_dir(sess), C.load_info(sess),
                           voxel=a.voxel)
        C.log(f"  3D 맵 {r['points']:,} pts · rgb={r['rgb']} · {r['size_mb']} MB "
              f"-> {dst / 'map_3d.ply'}")
    C.log(f"완료 — {dst} ({len(zup)} poses, 관측 {g['observations']}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
