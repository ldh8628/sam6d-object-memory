#!/usr/bin/env python3
"""SDK 원본 -> 알고리즘 입력용 표준 bag. 이 스택의 **유일한 변환 진입점**이다.

한 세션(= SAM 측면카메라 + SLAM 정면카메라 한 쌍)에 대해 아래를 전부 처리한다.

    1 clock      두 호스트의 시계차를 실측한다 (직접지정 -> genlock -> manifest -> 표 -> 0)
    2 convert    두 카메라를 depth 정렬된 4토픽 표준 bag 으로 만들고, 시계차를 SAM 에 굽는다
    3 lidar      Velodyne bag 의 rosbag2 metadata 를 v9 -> v5 로 낮추고 payload 를 복사한다
    4 imu        Xsens bag 을 그대로 복사한다 (이미 표준 v5)

산출 폴더에는 rosbag2 가 요구하는 파일과 info.json 하나만 남는다.

    <출력>/<세션>/
        SAM/    metadata.yaml + SAM_0.db3
        SLAM/   metadata.yaml + SLAM_0.db3
        lidar/  metadata.yaml(v5) + *.db3   (Velodyne 이 있을 때만)
        imu/    metadata.yaml     + *.db3   (Xsens 가 있을 때만)
        info.json                                             ← 메타데이터는 전부 여기

사용:
    python3 converting_launch/convert_all.py \
        --inputs:=data_slam/260807_chungbuk \
        --outputs:=data_slam_converted/260807_chungbuk

    # 한 세션만, 카메라 두 대만
    python3 converting_launch/convert_all.py \
        --inputs:=data_slam/260807_chungbuk/185223__circle_ccw_8laps \
        --outputs:=data_slam_converted/260807_chungbuk/185223__circle_ccw_8laps \
        --stages:=clock,convert
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

# 시스템 python3 에는 yaml/numpy 가 없다. 없으면 있는 인터프리터로 자기 자신을 다시 실행한다.
try:
    import yaml
    import numpy as np
except ModuleNotFoundError:                                    # pragma: no cover
    _py = Path.home() / "miniconda3" / "envs" / "sam6d_ros_humble" / "bin" / "python"
    if _py.is_file() and Path(sys.executable).resolve() != _py.resolve():
        os.execv(str(_py), [str(_py)] + sys.argv)
    raise

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

CONVERT_RECORDING = HERE / "convert_recording.py"
CALIB_OFFSET = HERE / "calib_offset.py"
DOWNGRADE = HERE / "downgrade_bag_metadata.py"
KNOWN_CLOCKS = HERE / "known_clock_offsets.json"

CONDA_SH = Path.home() / "miniconda3" / "etc" / "profile.d" / "conda.sh"
PY_NUMPY = Path.home() / "miniconda3" / "envs" / "sam6d_ros_humble" / "bin" / "python"
ENV_ROSBAG = "sam6d_ros_humble"          # rosbag2_py 가 있는 환경 (쓰기용)

STAGES = ("clock", "convert", "lidar", "imu")
COLOR_TOPIC = "/camera/camera/color/image_raw"


# --------------------------------------------------------------------------
# 잡동사니
# --------------------------------------------------------------------------
def ros2_argv(argv):
    """ROS2 스타일 `--key:=value` 를 argparse 가 아는 `--key value` 로 편다."""
    out = []
    for a in argv:
        if a.startswith("--") and ":=" in a:
            k, v = a.split(":=", 1)
            out += [k, v]
        else:
            out.append(a)
    return out


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def rel(p) -> str:
    """레포 안이면 상대경로로, 밖이면 절대경로 그대로 기록한다."""
    p = Path(p)
    try:
        return str(p.resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


def run(cmd, log_path=None, timeout=14400, live=False):
    """셸 명령 하나. (returncode, stdout, stderr)"""
    if isinstance(cmd, (list, tuple)):
        cmd = [str(c) for c in cmd]
        shell = False
    else:
        cmd, shell = ["bash", "-lc", cmd], False
    p = subprocess.run(cmd, cwd=str(ROOT), capture_output=not live, text=True, timeout=timeout)
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(log_path).write_text((p.stdout or "") + (p.stderr or ""), encoding="utf-8")
    return p.returncode, p.stdout or "", p.stderr or ""


def conda(env, cmd, **kw):
    return run(f"source {CONDA_SH} && conda activate {env} && {cmd}", **kw)


def read_metadata(bag_dir: Path) -> dict | None:
    p = Path(bag_dir) / "metadata.yaml"
    if not p.is_file():
        return None
    return yaml.safe_load(p.read_text())["rosbag2_bagfile_information"]


def bag_summary(bag_dir: Path) -> dict:
    """토픽별 메시지 수 · 길이 · 시작시각. rosbag2 를 열지 않고 metadata 만 읽는다."""
    m = read_metadata(bag_dir)
    if m is None:
        return {}
    topics = {e["topic_metadata"]["name"]: e["message_count"]
              for e in m.get("topics_with_message_count", [])}
    return {"topics": topics,
            "duration_s": round(m["duration"]["nanoseconds"] / 1e9, 3),
            "t_start_ns": m["starting_time"]["nanoseconds_since_epoch"],
            "version": m.get("version"),
            "files": [f["path"] for f in m.get("files", [])]}


def enrich_role(r: dict) -> dict:
    """다운스트림(gridmap / object_lifecycle / run_integration)이 기대하는 필드를 채운다.

    변환기가 주는 것은 K·D·color_size 같은 납작한 값이라, 그걸로 camera 절과 토픽 이름을
    만들어 준다. 토픽은 우리 4토픽 규격이라 고정이다.
    """
    if not isinstance(r, dict) or r.get("status") == "failed":
        return r
    t = r.get("topics") or {}
    r.setdefault("color_topic", COLOR_TOPIC)
    r.setdefault("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
    r.setdefault("caminfo_topic", "/camera/camera/color/camera_info")
    n = t.get(COLOR_TOPIC)
    if n is not None:
        r["color_count"] = n
        d = r.get("duration_s") or 0
        r["color_hz"] = round(n / d, 2) if d else None
    K, size = r.get("K"), r.get("color_size")
    if K and size:
        r["camera"] = {"width": int(size[0]), "height": int(size[1]),
                       "K": list(K), "D": list(r.get("D") or [0.0] * 5),
                       "distortion_model": "plumb_bob"}
    return r


def pick_db3(d: Path):
    c = sorted(p for p in Path(d).iterdir() if p.suffix == ".db3")
    return c[0] if len(c) == 1 else None


# --------------------------------------------------------------------------
# 세션 탐색
# --------------------------------------------------------------------------
def is_session(d: Path) -> bool:
    """SAM/ 또는 SLAM/ 을 직접 품고 있으면 세션 폴더다."""
    return (d / "SLAM").is_dir() or (d / "SAM").is_dir()


def find_sessions(inputs: Path, only=None):
    inputs = inputs.resolve()
    if is_session(inputs):
        return [inputs]
    subs = sorted(d for d in inputs.iterdir() if d.is_dir() and is_session(d))
    if only:
        want = set(only)
        subs = [d for d in subs if d.name in want]
        missing = want - {d.name for d in subs}
        if missing:
            raise SystemExit(f"--session 으로 준 이름을 찾을 수 없다: {sorted(missing)}")
    return subs


def velodyne_dir(slam_src: Path):
    for pat in ("cpr-*velodyne*", "velodyne", "*velodyne*"):
        for c in sorted(slam_src.glob(pat)):
            if c.is_dir() and (c / "metadata.yaml").is_file():
                return c
    return None


def xsens_dir(slam_src: Path):
    for pat in ("xsens", "*xsens*"):
        for c in sorted(slam_src.glob(pat)):
            if c.is_dir() and (c / "metadata.yaml").is_file():
                return c
    return None


# --------------------------------------------------------------------------
# 1) clock — 두 호스트 시계차
# --------------------------------------------------------------------------
def manifest_start_ns(sess_dir: Path):
    """레코더가 남긴 '예정 시작시각'. 사이드카 이름이 데이터셋마다 다르다."""
    for name in ("session_manifest.json", "recording_summary.json"):
        p = sess_dir / name
        if not p.is_file():
            continue
        try:
            j = json.load(open(p, encoding="utf-8"))
        except ValueError:
            continue
        for k in ("scheduled_capture_start_time_ns", "scheduled_start_time_ns",
                  "start_time_ns", "first_timestamp_ns", "recording_start_time_ns"):
            if isinstance(j.get(k), int):
                return j[k], f"{name}:{k}"
    return None, None


def known_clock_offset(dataset: str, session: str) -> dict | None:
    """사람이 재서 known_clock_offsets.json 에 넣어 둔 값. 세션 키가 날짜 키보다 우선한다."""
    if not KNOWN_CLOCKS.is_file():
        return None
    try:
        t = json.loads(KNOWN_CLOCKS.read_text(encoding="utf-8"))
    except ValueError:
        return None
    for k in (f"{dataset}/{session}", dataset):
        e = t.get(k)
        if isinstance(e, dict) and isinstance(e.get("offset_ns"), int):
            return {**e, "key": k}
    return None


def stage_clock(slam_src: Path, sam_src: Path, cache: Path,
                dataset: str, session: str, manual_ns) -> dict:
    """SAM 타임스탬프에서 '빼야 할' 오프셋(ns): t_common = t_SAM - offset_ns.

    실패해도 0 으로 진행하되, 무엇으로 구했는지는 반드시 method 에 남긴다.
    """
    if not sam_src.is_dir():
        return {"offset_ns": 0, "method": "no_sam_camera",
                "applied_to": None, "note": "SAM 카메라가 없는 세션"}

    # 0순위 — 사람이 직접 지정
    if manual_ns is not None:
        return {"offset_ns": int(manual_ns), "method": "manual_cli",
                "applied_to": "SAM", "note": "--clock-offset-ns 로 직접 지정한 값"}

    # 1순위 — genlock Global Time 실측
    rc, out, err = run([PY_NUMPY, CALIB_OFFSET, "--slam", slam_src, "--sam", sam_src,
                        "--stream", "color", "--cache", cache])
    if rc == 0 and out.strip().lstrip("-").isdigit():
        r = {}
        if Path(cache).is_file():
            key = f"{slam_src.resolve()}|{sam_src.resolve()}|color"
            r = json.load(open(cache, encoding="utf-8")).get(key, {})
        detail = [l for l in err.splitlines() if "=>" in l]
        return {"offset_ns": int(out.strip()),
                "method": r.get("method", "calib_offset_genlock"),
                "applied_to": "SAM",
                "probe_source": r.get("probe_source"),
                "probe_ms": r.get("probe_ms"), "residual_ms": r.get("residual_ms"),
                "sd_ms": r.get("sd_ms"), "n": r.get("n"),
                "detail": detail[0].strip() if detail else r.get("note", "")}
    first_fail = err.strip().splitlines()[0] if err.strip() else f"rc={rc}"

    # 2순위 — 레코더 manifest 의 예정 시작시각 차
    a, ka = manifest_start_ns(slam_src)
    b, kb = manifest_start_ns(sam_src)
    if a is not None and b is not None:
        return {"offset_ns": int(b - a), "method": "manifest_scheduled_start",
                "applied_to": "SAM", "detail": f"{kb} - {ka}",
                "note": f"genlock 실측 실패({first_fail}) — manifest 로 대체"}

    # 3순위 — 사람이 미리 재서 표에 넣어 둔 값
    k = known_clock_offset(dataset, session)
    if k is not None:
        return {"offset_ns": int(k["offset_ns"]),
                "method": k.get("method", "known_table"), "applied_to": "SAM",
                "table_key": k["key"], "source": k.get("source"),
                "detail": k.get("evidence", ""),
                "note": f"genlock 실측 실패({first_fail}), manifest 없음 — "
                        f"known_clock_offsets.json 의 값을 쓴다"}

    # 4순위 — 모른다. 0 으로 두되 반드시 티가 나게 남긴다.
    return {"offset_ns": 0, "method": "unmeasured", "applied_to": None,
            "note": f"genlock 실측 실패({first_fail}), manifest 없음, "
                    f"known_clock_offsets.json 에도 없음 — 두 카메라 시계가 "
                    f"어긋나 있다면 융합 결과가 틀어진다. 값을 재서 "
                    f"--clock-offset-ns 로 주거나 표에 넣을 것"}


# --------------------------------------------------------------------------
# 2) convert — 카메라 두 대
# --------------------------------------------------------------------------
def bag_is_complete(dst: Path, src: Path) -> bool:
    """metadata.yaml 이 있다고 완성본이 아니다. 원본 프레임 수와 맞는지 본다.

    (기존 integration/pipeline/convert.py 는 존재만 보고 건너뛰어서, --stride/--max 로
     잘라 만든 bag 을 조용히 재사용하는 버그가 있었다.)
    """
    s = bag_summary(dst)
    if not s or COLOR_TOPIC not in s.get("topics", {}):
        return False
    assoc = src / "rgbd_timestamp_associations.json"
    if not assoc.is_file():
        return True                       # 대조할 근거가 없으면 존재만으로 인정
    try:
        n = len(json.load(open(assoc, encoding="utf-8"))["associations"])
    except Exception:                                          # noqa: BLE001
        return True
    return s["topics"][COLOR_TOPIC] >= n - 1   # 변환기가 마지막 1프레임을 버릴 수 있다


def convert_camera(src: Path, dst: Path, offset_ns: int, stride: int,
                   label: str, logdir: Path, force: bool) -> dict:
    if dst.exists() and not force and bag_is_complete(dst, src):
        log(f"  {label}: 이미 완성본 — 건너뜀")
    else:
        if dst.exists():
            if not force:
                raise RuntimeError(f"기존 불완전 변환본을 보존했다: {dst} (--force 로 재생성)")
            shutil.rmtree(dst)
        dst.parent.mkdir(parents=True, exist_ok=True)
        # color_global 이 정확하지만 metadata 토픽이 없는 데이터셋이 있어 assoc 으로 폴백한다.
        last = ""
        for ts in ("color_global", "assoc"):
            command = ["python", "-O", str(CONVERT_RECORDING), str(src), str(dst),
                       "--time-source", ts, "--offset-ns", str(offset_ns)]
            if stride > 1:
                command += ["--stride", str(stride)]
            log_path = logdir / f"convert_{label}_{ts}.log"
            rc, out, err = conda(
                ENV_ROSBAG,
                f"set -o pipefail && {shlex.join(command)} 2>&1 | "
                f"tee {shlex.quote(str(log_path))}",
                live=True)
            if rc == 0 and (dst / "metadata.yaml").is_file():
                break
            output = log_path.read_text(encoding="utf-8") if log_path.is_file() else err or out
            last = output.strip().splitlines()[-1] if output.strip() else f"rc={rc}"
            if dst.exists():
                shutil.rmtree(dst)
            log(f"  {label}: time-source={ts} 실패 ({last}) — 폴백")
        else:
            raise RuntimeError(f"{label} 변환 실패 ({src}): {last}")

    s = bag_summary(dst)
    # 변환기가 bag 안에 남긴 사이드카는 info.json 으로 흡수하고 지운다.
    ci = dst / "conversion_info.json"
    if ci.is_file():
        try:
            s.update({k: v for k, v in json.load(open(ci, encoding="utf-8")).items()
                      if k not in ("topics",)})
        except ValueError:
            pass
        ci.unlink()
    s["path"] = dst.name
    s["frames"] = s.get("topics", {}).get(COLOR_TOPIC)
    s["source"] = rel(src)
    return s


# --------------------------------------------------------------------------
# 3~4) lidar / imu — 재인코딩 없이 복사한다
# --------------------------------------------------------------------------
def copy_bag(src: Path, dst: Path, force: bool) -> dict:
    """metadata 만 v5 로 맞추고 payload(.db3)는 원본에서 복사한다.

    변환본은 원본을 안 보고도 홀로 서야 하므로 심볼릭이 아니라 실데이터를 둔다.
    payload 는 재인코딩하지 않으므로 바이트가 원본과 같다(velodyne 은 metadata 의
    v9->v5 만 바뀐다). 30세션 전량이면 lidar+imu 로 약 45.6 GB 를 더 쓴다.
    """
    m = read_metadata(src)
    if m is None:
        raise RuntimeError(f"{src}: metadata.yaml 이 없다")
    src_version = m.get("version")

    if dst.exists():
        if not force and (dst / "metadata.yaml").is_file():
            # 다시 만들지는 않되, 기록은 처음 만들 때와 같은 모양으로 돌려준다.
            # downgraded_from 은 **원본** 버전에서 읽는다 — 이미 낮춰 둔 dst 를 보면 언제나
            # v5 라서 'v9 였다'는 내력이 지워진다.
            r = {**bag_summary(dst), "path": dst.name, "source": rel(src),
                 "copied_files": sorted(f.name for f in dst.iterdir()
                                        if f.suffix == ".db3"),
                 "note": "이미 있음 — 다시 만들지 않았다"}
            if src_version and src_version > 5:
                r["downgraded_from"] = f"v{src_version}"
            return r
        if not force:
            raise RuntimeError(f"기존 불완전 복사본을 보존했다: {dst} (--force 로 재생성)")
        shutil.rmtree(dst) if not dst.is_symlink() else dst.unlink()

    if src_version and src_version > 5:
        tmp = dst.parent / "_downgrade_tmp"
        shutil.rmtree(tmp, ignore_errors=True)
        rc, out, err = run([PY_NUMPY, DOWNGRADE, src, "--out-root", tmp, "--no-verify"])
        made = tmp / src.name
        if rc != 0 or not (made / "metadata.yaml").is_file():
            shutil.rmtree(tmp, ignore_errors=True)
            raise RuntimeError(f"metadata v{src_version}->v5 실패 ({src}): "
                               f"{(err or out).strip()[-300:]}")
        made.rename(dst)
        shutil.rmtree(tmp, ignore_errors=True)
    else:
        dst.mkdir(parents=True)
        shutil.copyfile(src / "metadata.yaml", dst / "metadata.yaml")

    # sqlite WAL 이 체크포인트되지 않은 채 끝난 bag 이면 레코드가 -wal 에 남아 있어
    # .db3 만 가져오면 조용히 비어 버린다. 지금 원본은 22개 전부 0바이트(=체크포인트됨)라
    # .db3 하나로 충분하지만, 다음 촬영에서 다시 생길 수 있으므로 발견하면 멈춘다.
    for w in sorted(src.glob("*.db3-wal")):
        if w.stat().st_size > 0:
            raise RuntimeError(
                f"{w}: WAL 이 {w.stat().st_size} 바이트 남아 있다. .db3 만 복사하면 "
                f"데이터가 빈다 — 원본을 sqlite3 로 체크포인트한 뒤 다시 변환해라")

    copied = []
    for f in sorted(src.glob("*.db3")):
        t = dst / f.name
        if t.is_symlink() or t.exists():
            t.unlink()
        shutil.copyfile(f, t)
        copied.append(f.name)

    left = [f.name for f in dst.iterdir() if f.is_symlink()]
    if left:
        raise RuntimeError(f"{dst}: 심볼릭이 남았다 {left} — 변환본은 실데이터만 둔다")

    s = bag_summary(dst)
    s.update({"path": dst.name, "copied_files": copied, "source": rel(src)})
    if src_version and src_version > 5:
        s["downgraded_from"] = f"v{src_version}"
    return s


# --------------------------------------------------------------------------
# 세션 하나
# --------------------------------------------------------------------------
def load_prev_clock(dst: Path) -> dict:
    """전에 돌려 둔 info.json 의 clock 절. --stages 로 단계를 쪼개 돌릴 때 필요하다."""
    p = dst / "info.json"
    if p.is_file():
        try:
            return json.loads(p.read_text(encoding="utf-8")).get("clock") or {}
        except ValueError:
            pass
    return {}


def convert_session(src: Path, dst: Path, a) -> dict:
    dataset = src.parent.name
    session = src.name
    slam_src, sam_src = src / "SLAM", src / "SAM"
    # 로그·중간 산출물은 출력 폴더 밖에 둔다 — 출력에는 bag 과 info.json 만 남긴다.
    logdir = ROOT / "integration" / "output" / dataset / session / "logs_convert"
    dst.mkdir(parents=True, exist_ok=True)
    logdir.mkdir(parents=True, exist_ok=True)

    # 단계를 쪼개 돌려도 앞 단계의 결과를 잃지 않게 기존 info.json 위에 얹는다.
    info = {}
    if (dst / "info.json").is_file():
        try:
            info = json.loads((dst / "info.json").read_text(encoding="utf-8"))
        except ValueError:
            info = {}
    info.update({"session": session, "dataset": dataset,
                 "generated_by": "converting_launch/convert_all.py",
                 "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                 "source": rel(src)})
    # 단계를 쪼개 돌려도 '이 세션에 무엇이 돌았는지' 기록이 사라지지 않게 누적한다.
    info["stages_run"] = [x for x in (info.get("stages_run") or []) if x not in a.stages]

    # ---- 1 clock
    if "clock" in a.stages:
        info["clock"] = stage_clock(slam_src, sam_src, HERE / "clock_offsets.json",
                                    dataset, session, a.clock_offset_ns)
        info["stages_run"].append("clock")
        c = info["clock"]
        log(f"  clock: {c['offset_ns'] / 1e6:+.3f} ms  [{c['method']}]"
            + (f"  ({c['note']})" if c.get("note") else ""))
    off = int(info.get("clock", {}).get("offset_ns", 0))

    # ---- 2 convert
    if "convert" in a.stages:
        # 한 카메라가 죽어도 나머지는 살린다 (0724 eight3b 처럼 원본 db3 가 0 바이트인 세션이 있다).
        for role, src_dir, o in (("SLAM", slam_src, 0), ("SAM", sam_src, -off)):
            if not src_dir.is_dir():
                continue
            role_dst = dst / role
            existed = role_dst.exists()
            try:
                info[role] = convert_camera(src_dir, role_dst, o, a.stride,
                                            role, logdir, a.force)
                extra = f" · offset {o / 1e6:+.3f} ms 적용" if role == "SAM" else ""
                log(f"  {role:5s}: {info[role].get('frames')} 프레임 · "
                    f"{info[role].get('duration_s')} s{extra}")
            except Exception as exc:                            # noqa: BLE001
                info[role] = {"status": "failed", "reason": str(exc)[:400],
                              "source": rel(src_dir)}
                if role_dst.exists() and (a.force or not existed):
                    shutil.rmtree(role_dst)
                log(f"  {role:5s}: 변환 실패 — {str(exc)[:200]}")
        info["stages_run"].append("convert")

    # ---- 3 lidar
    if "lidar" in a.stages:
        v = velodyne_dir(slam_src) if slam_src.is_dir() else None
        if v is None:
            info["lidar"] = None
            log("  lidar: Velodyne bag 없음 — hdl 은 이 세션에서 못 돌린다")
        else:
            info["lidar"] = copy_bag(v, dst / "lidar", a.force)
            info["lidar"]["topic"] = "/velodyne_points"
            log(f"  lidar: {info['lidar'].get('downgraded_from', 'v5')} -> v5, "
                f"{info['lidar'].get('duration_s')} s")
        info["stages_run"].append("lidar")

    # ---- 4 imu
    if "imu" in a.stages:
        x = xsens_dir(slam_src) if slam_src.is_dir() else None
        if x is None:
            info["imu"] = None
            log("  imu  : Xsens bag 없음")
        else:
            info["imu"] = copy_bag(x, dst / "imu", a.force)
            info["imu"]["topic"] = "/xsens/imu/data"
            log(f"  imu  : {info['imu'].get('duration_s')} s")
        info["stages_run"].append("imu")

    for role in ("SLAM", "SAM"):
        if role in info:
            info[role] = enrich_role(info[role])
    (dst / "info.json").write_text(json.dumps(info, indent=1, ensure_ascii=False),
                                   encoding="utf-8")
    return info


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inputs", required=True,
                    help="SDK 원본. 날짜 폴더 또는 단일 세션 폴더")
    ap.add_argument("--outputs",
                    help="변환본을 쓸 곳 (기본: output/<inputs 폴더명>/converted)")
    ap.add_argument("--session", default="",
                    help="쉼표로 구분한 세션 이름 (날짜 폴더를 줬을 때 일부만)")
    ap.add_argument("--stages", default=",".join(STAGES))
    ap.add_argument("--stride", type=int, default=1,
                    help="N 프레임마다 하나만. 기본 1 = 전부 (희소본은 만들지 않는다)")
    ap.add_argument("--clock-offset-ns", type=int, default=None,
                    help="두 호스트 시계차를 직접 지정한다 [ns]. t_common = t_SAM - 이 값. "
                         "주면 실측·표를 모두 제치고 이 값을 쓴다")
    ap.add_argument("--force", action="store_true", help="이미 있어도 다시 만든다")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(ros2_argv(sys.argv[1:]))
    if a.stride < 1:
        ap.error("--stride 는 1 이상이어야 한다")

    a.stages = [s.strip() for s in a.stages.split(",") if s.strip()]
    bad = [s for s in a.stages if s not in STAGES]
    if bad:
        raise SystemExit(f"모르는 단계: {bad}  (쓸 수 있는 것: {list(STAGES)})")

    if a.inputs == "" or a.outputs == "":
        ap.error("--inputs/--outputs 값이 비어 있다")
    inputs = Path(a.inputs).expanduser().resolve()
    outputs = (Path(a.outputs).expanduser() if a.outputs is not None else
               ROOT / "output" / inputs.name / "converted").resolve()
    if not inputs.is_dir():
        raise SystemExit(f"--inputs 가 폴더가 아니다: {inputs}")
    only = [s.strip() for s in a.session.split(",") if s.strip()] or None
    sess = find_sessions(inputs, only)
    if not sess:
        raise SystemExit(f"{inputs} 아래에서 SAM/·SLAM/ 을 품은 세션을 찾지 못했다")

    single = is_session(inputs)
    jobs = [(s, outputs if single else outputs / s.name) for s in sess]
    for s, d in jobs:
        if s == d:
            ap.error(f"입력과 출력이 같다: {s}")
        info_path = d / "info.json"
        if info_path.is_file() and not a.force:
            try:
                old_source = json.loads(info_path.read_text(encoding="utf-8")).get("source")
            except ValueError:
                old_source = None
            if old_source and old_source != rel(s):
                ap.error(f"기존 출력의 원본이 다르다: {d} ({old_source} != {rel(s)})")
    log(f"세션 {len(sess)}개 · 단계 {a.stages} · stride {a.stride}")
    for s in sess:
        log(f"  - {s.name}")
    if a.dry_run:
        for s, d in jobs:
            print(f"{s}  ->  {d}")
        return 0

    ok, failed, skipped = [], [], []
    for i, (s, d) in enumerate(jobs, 1):
        # convert 를 안 돌리는 실행(예: --stages:=imu 로 info.json 만 다시 쓰기)에서
        # 아직 변환하지 않은 세션까지 건드리면 카메라 bag 없는 껍데기 폴더가 생긴다.
        if "convert" not in a.stages and not any((d / r).is_dir() for r in ("SLAM", "SAM")):
            skipped.append(s.name)
            continue
        log(f"[{i}/{len(sess)}] {s.name}  ->  {d}")
        try:
            convert_session(s, d, a)
            ok.append(s.name)
        except Exception as exc:                                # noqa: BLE001
            failed.append((s.name, str(exc)[:300]))
            log(f"  !! 실패: {str(exc)[:300]}")
    if skipped:
        log(f"건너뜀 {len(skipped)}개 — 아직 변환하지 않은 세션이라 손대지 않았다 "
            f"(convert 단계를 넣어 돌리면 만들어진다): {', '.join(skipped[:6])}"
            + (" …" if len(skipped) > 6 else ""))
    log(f"완료 {len(ok)}/{len(sess) - len(skipped)}")
    for n, e in failed:
        log(f"  실패 {n}: {e}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
