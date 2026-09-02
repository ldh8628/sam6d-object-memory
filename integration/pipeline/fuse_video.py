#!/usr/bin/env python3
"""4단계 — SLAM 3종 각각에 SAM-6D 결과를 합치고, 확인용 동영상을 만든다.

    python integration/pipeline/fuse_video.py 185223__circle_ccw_8laps

백엔드마다 다음을 만든다 (<출력>/fused/<백엔드>/):

    objects.json   map 프레임의 지속 객체 목록 (위치·자세·존재확률)
    map_2d.png     그 백엔드의 2D 맵 위에 객체를 얹은 그림
    video.mp4      위: SLAM 카메라 영상 | SAM 카메라 영상 | SAM 영상 + 3D 박스
                   아래: 2D 맵 위의 두 카메라 현재 위치·방향 + 객체 (검출 중이면 채움)

융합 식:  T_map_obj = T_map_camSLAM(t) · X · T_camSAM_obj
X = T_camSLAM_camSAM 는 두 카메라의 리그 변환이며 반드시 있어야 한다. 없으면 어디서
구하는지 안내하고 멈춘다 — 추정하려면 SAM bag 에 ORB3 를 한 번 더 돌려야 하고, 그건
이 파이프라인이 말없이 만들 산출물이 아니기 때문이다.

hdl 은 LiDAR 프레임이라 여기서 T_velo_cam 을 ORB3 궤적과의 hand-eye 로 풀어 카메라
프레임으로 옮긴다. **캘리브가 아니라 궤적 정합이므로 hdl 융합 결과는 ORB3 에 부분적
으로 의존한다.**

마지막에 중간물(<출력>/_stage/, PEM 입력·오버레이 포함)을 지운다. --keep-stage 로 남길 수 있다.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

# common 을 먼저 불러야 한다 — numpy 가 없는 인터프리터로 실행됐을 때 자기 자신을
# 다시 실행하는 shim 이 common 안에 있다. numpy 를 위에 두면 그 전에 죽는다.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import common as C  # noqa: E402
import gridmap as GM  # noqa: E402

import numpy as np  # noqa: E402

RUN_INTEGRATION = C.INTEGRATION_DIR / "run_integration.py"
MAKE_REVIEW = C.INTEGRATION_DIR / "make_review.py"
CAD_EXTENTS = C.INTEGRATION_DIR / "cad_extents.json"
BACKENDS = ("orb3", "rtabmap", "hdl")


def video_rotation(backend: str) -> np.ndarray:
    """백엔드 map 프레임 -> make_review 가 가정하는 광학 프레임(x 오른쪽, z 앞).

    make_review 의 지도 패널은 x-z 평면을 위에서 본 것으로 그린다. ORB3/RTAB 의 map 은
    이미 광학 프레임이라 그대로지만, hdl 은 z-up 이라 그대로 넘기면 지도가 '수직 단면'
    으로 나온다. 그래서 hdl 만 z-up -> 광학 으로 돌려서 넘긴다.
    """
    return np.eye(3) if backend in ("orb3", "rtabmap") else C.R_OPT2ROS.T


def rotate_tum(src: Path, dst: Path, R: np.ndarray, X: np.ndarray | None = None) -> None:
    """TUM 궤적에 map 회전 G 를 왼쪽에서, (선택) 카메라 변환 X 를 오른쪽에서 곱한다."""
    ts, xyz, q = C.read_tum(src)
    rows = []
    for t, p, qq in zip(ts, xyz, q):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = C.quat_to_R(qq), p
        G = np.eye(4)
        G[:3, :3] = R
        T = G @ T
        if X is not None:
            T = T @ X
        o = C.R_to_quat(T[:3, :3])
        rows.append(f"{t:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} "
                    f"{o[0]:.6f} {o[1]:.6f} {o[2]:.6f} {o[3]:.6f}")
    dst.write_text("\n".join(rows) + "\n")


def write_pcd(pts: np.ndarray, dst: Path, max_pts: int = 50_000) -> Path:
    if pts.shape[0] > max_pts:
        pts = pts[np.random.default_rng(0).choice(pts.shape[0], max_pts, replace=False)]
    body = "\n".join(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in pts)
    dst.write_text("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
                   "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                   f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                   f"POINTS {len(pts)}\nDATA ascii\n" + body + "\n")
    return dst


def rotate_object_map(src: Path, dst: Path, R: np.ndarray) -> Path:
    om = json.loads(src.read_text())
    for o in om["objects"]:
        o["xyz"] = list(R @ np.asarray(o["xyz"], float))
    dst.write_text(json.dumps(om, indent=1))
    return dst


def hdl_to_camera_frame(stage: Path, out: Path) -> None:
    """hdl 의 LiDAR 궤적을 ORB3 와의 hand-eye 로 카메라 프레임으로 옮긴다.

    _stage 가 이미 지워졌어도 보존된 <출력>/{hdl,orb3}/trajectory.txt 만으로 다시 풀 수
    있게 입력을 되채운다. 그래야 영상만 다시 뽑을 때 hdl 을 10분 재실행하지 않는다.
    """
    d = stage / "slam_hdl"
    d.mkdir(parents=True, exist_ok=True)
    velo = d / "trajectory_velo_dense.txt"
    if not velo.is_file() and (out / "hdl" / "trajectory.txt").is_file():
        shutil.copyfile(out / "hdl" / "trajectory.txt", velo)
    orb = stage / "slam_orb3" / "trajectory.txt"
    if not orb.is_file():
        orb = out / "orb3" / "trajectory.txt"
    if not orb.is_file():
        C.die("hdl 융합에는 ORB3 궤적이 필요하다 (T_velo_cam 을 hand-eye 로 풀어야 한다). "
              "먼저 orb3.py 를 돌려라.")
    C.run(f"{C.CONDA} run -n hdl_graph_slam_humble --no-capture-output python -c "
          f"\"import sys; sys.path.insert(0, r'{C.TOOLS_ROOT}'); from pathlib import Path; "
          f"import densify_hdl_0807 as D; D.to_camera_frame(Path(r'{d}'), Path(r'{orb}'))\"",
          log_path=d / "handeye.log", label="hdl hand-eye (T_velo_cam)")
    if not (d / "trajectory.txt").is_file():
        C.die(f"hdl hand-eye 실패 — 로그: {d / 'handeye.log'}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--only", nargs="+", choices=BACKENDS, default=list(BACKENDS))
    ap.add_argument("--res", type=float, default=0.05, help="격자 해상도 [m/cell]")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--keep-stage", action="store_true", help="중간물을 지우지 않는다")
    a = ap.parse_args()

    sess = C.resolve_session(a.session)
    data, out = C.data_dir(sess), C.out_dir(sess)
    stage = out / "_stage"
    have_pem = (stage / "sam" / "pem_inputs" / "manifest.csv").is_file()
    if not have_pem and not (out / "fused").is_dir():
        C.die(f"SAM-6D 중간물이 없다 ({stage}/sam/pem_inputs) — sam6d.py 를 먼저 돌려라. "
              "이미 돌렸다면 fuse_video.py 가 지운 뒤라 다시 돌려야 한다.")

    Xt, x_src = C.load_rig_extrinsic(data)
    if Xt is None:
        C.die(f"리그 extrinsic 을 못 찾았다 — {x_src}\n"
              f"  X = T_camSLAM_camSAM 는 두 카메라의 리그 변환이고, 정본은\n"
              f"  converting_launch/rigs/<날짜>.urdf 의 slam_cam_to_sam_cam 조인트다.\n"
              f"  이 데이터셋 URDF 가 없으면 만들어야 한다 — 변환기는 extrinsic 을 "
              f"추정하지 않는다(ORB3 궤적에 종속되기 때문).")
    X = np.array(Xt, dtype=float)
    C.log(f"extrinsic X = {x_src}")

    done = []
    for b in a.only:
        traj = out / b / "trajectory.txt"
        if not traj.is_file():
            C.log(f"건너뜀 {b}: {traj} 가 없다 ({b}.py 를 먼저 돌려라)")
            continue
        C.log(f"===== {b} =====")
        d = stage / f"slam_{b}"
        d.mkdir(parents=True, exist_ok=True)
        dst = out / "fused" / b
        dst.mkdir(parents=True, exist_ok=True)
        if b == "hdl":
            hdl_to_camera_frame(stage, out)     # -> d/trajectory.txt (카메라 프레임)
        elif not (d / "trajectory.txt").is_file():
            shutil.copyfile(traj, d / "trajectory.txt")

        # ---- 융합 ---------------------------------------------------------
        fused = stage / f"fused_{b}"
        if not (fused / "object_map.json").is_file() and not (dst / "objects.json").is_file():
            C.run(f"python3 {RUN_INTEGRATION} {data} --stage fuse --out {stage} "
                  f"--slam-dir slam_{b} --fused-dir fused_{b} --extrinsic {xs[0]}",
                  log_path=out / "logs" / f"fuse_{b}.log", label=f"{b} 융합")
        if (fused / "object_map.json").is_file():
            shutil.copyfile(fused / "object_map.json", dst / "objects.json")
        if not (dst / "objects.json").is_file():
            C.log(f"!! {b} 융합 실패 — 로그: {out / 'logs' / f'fuse_{b}.log'}")
            continue
        om = json.loads((dst / "objects.json").read_text())
        live = [o for o in om["objects"] if o["status"] != "deleted"]
        C.log(f"{b}: 객체 {len(live)}개 생존")

        # ---- 2D 맵 + 객체 --------------------------------------------------
        # 백엔드가 만든 격자를 그대로 읽어 객체만 얹는다. 점군에서 다시 만들면 융합
        # 그림이 <출력>/<백엔드>/map_2d 와 미묘하게 어긋나고, 점군은 _stage 에만 있다.
        grid = GM.load_map(out / b / "map_2d")
        Rm = C.map_rotation(b)
        _t, xyz, _q = C.read_tum(d / "trajectory.txt")
        C.render_map_png(grid, xyz @ Rm.T, dst / "map_2d.png",
                         f"{b} + SAM-6D ({len(live)} objects)",
                         [{"name": o["object_name"],
                           "position": (Rm @ np.asarray(o["xyz"], float))[:2]}
                          for o in live])

        # ---- 동영상 --------------------------------------------------------
        G = video_rotation(b)
        vtraj = d / "video_slam_traj.txt"
        vsam = d / "video_sam_traj.txt"
        rotate_tum(d / "trajectory.txt", vtraj, G)
        rotate_tum(d / "trajectory.txt", vsam, G, X)     # T_map_camSAM = T_map_camSLAM · X
        # 지도 패널 배경은 --grid(점유격자)가 맡으므로 점군 인자는 형식상 빈 것을 넘긴다.
        vpts = write_pcd(np.zeros((1, 3)), d / "video_map_points.pcd")
        vobj = rotate_object_map(dst / "objects.json", d / "video_objects.json", G)
        vdir = d / "video"
        # _stage 가 지워졌으면 보존본으로 대체한다 (내용은 같다)
        pem = stage / "sam" / "sam_objects.json"
        if not pem.is_file():
            pem = out / "sam" / "objects.json"
        binfo = stage / "bag_info.json"
        if not binfo.is_file():
            binfo = data / "info.json"           # convert.py 가 쓴 것, sam.camera.K 동일
        rc = C.run(f"{C.PY_NUMPY} {MAKE_REVIEW} --pair {sess} "
                   f"--slam-bag {data / 'slam'} --sam-bag {data / 'sam'} "
                   f"--slam-traj {vtraj} --sam-traj {vsam} --map-points {vpts} "
                   f"--object-map {vobj} --pem-json {pem} "
                   f"--bag-info {binfo} --extrinsic {xs[0]} "
                   f"--grid {out / b / 'map_2d.pgm'} "
                   f"--cad-extents {CAD_EXTENTS} --out-dir {vdir} --fps {a.fps}",
                   log_path=out / "logs" / f"video_{b}.log", label=f"{b} 동영상")
        mp4 = vdir / "sync_view.mp4"
        if rc == 0 and mp4.is_file():
            shutil.move(str(mp4), dst / "video.mp4")
            C.log(f"{b}: {dst / 'video.mp4'}")
            done.append(b)
        else:
            C.log(f"!! {b} 동영상 실패 — 로그: {out / 'logs' / f'video_{b}.log'}")

    if not a.keep_stage and len(done) == len(a.only):
        n = sum(1 for _ in stage.rglob("*"))
        shutil.rmtree(stage, ignore_errors=True)
        C.log(f"중간물 삭제: _stage/ ({n} 파일)")
    elif not a.keep_stage:
        C.log(f"일부 백엔드가 끝나지 않아 _stage/ 를 남긴다 (성공 {done})")

    C.log(f"완료 — {out}/fused/{{{','.join(done)}}}")
    return 0 if len(done) == len(a.only) else 1


if __name__ == "__main__":
    raise SystemExit(main())
