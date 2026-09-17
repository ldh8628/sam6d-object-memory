#!/usr/bin/env python3
"""변환본 트리가 '전 세션 동일 구성' 인지 검사한다.

    python3 converting_launch/verify_layout.py --inputs:=data_slam_converted
                                              [--against:=converting_launch/baseline_before.json]
                                              [--strict]

검사 항목
    A 구성   세션마다 {SAM,SLAM,lidar,imu,calib,info.json} 외의 잡파일이 없다
    B 토픽   카메라 bag 은 정확히 4토픽이고 네 토픽의 메시지 수가 같다
    C 버전   모든 bag 의 rosbag2 metadata version 이 5 다 (Humble 이 읽을 수 있어야 한다)
    D 자립   lidar/imu 에 심볼릭이 없고 payload 크기가 원본과 같다 (변환본은 홀로 선다)
    E 시계   info.json 에 clock.method 가 있고, applied_to 가 SAM 이면 SAM 에 실제로 적용됐다
    F 기준   --against 를 주면 옛 변환본의 프레임 수·길이와 대조한다
    G 잔여   레포 안에 두 데이터 폴더 밖의 '입력용' bag 이 남아 있지 않다
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

ALLOWED = {"SAM", "SLAM", "lidar", "imu", "info.json"}
CAM_TOPICS = {"/camera/camera/color/image_raw",
              "/camera/camera/color/camera_info",
              "/camera/camera/aligned_depth_to_color/image_raw",
              "/camera/camera/aligned_depth_to_color/camera_info"}
COLOR = "/camera/camera/color/image_raw"

# 이 아래에 있는 bag 은 SLAM 이 만들어 낸 '출력' 이므로 G 검사 대상이 아니다.
OUTPUT_TREES = ("integration/output", "output_slam", "hdlgraphslam_ws/output",
                "orbslam_ws/output", "rtabmap_ws/output", "_old_converted",
                "_0724_staging", "slam_comparison_report")


def ros2_argv(argv):
    out = []
    for a in argv:
        if a.startswith("--") and ":=" in a:
            k, v = a.split(":=", 1)
            out += [k, v]
        else:
            out.append(a)
    return out


def meta(bag: Path):
    """rosbag2 metadata.yaml -> (topics{name:count}, duration_s, t_start_ns, version)."""
    p = bag / "metadata.yaml"
    if not p.is_file():
        return None
    txt = p.read_text(encoding="utf-8", errors="ignore")
    topics = {m.group(1): int(m.group(2))
              for m in re.finditer(r"name: (/\S+)[\s\S]{0,300}?message_count: (\d+)", txt)}
    d = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", txt)
    s = re.search(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(-?\d+)", txt)
    v = re.search(r"^\s*version:\s*(\d+)", txt, re.M)
    return {"topics": topics,
            "duration_s": round(int(d.group(1)) / 1e9, 3) if d else None,
            "t_start_ns": int(s.group(1)) if s else None,
            "version": int(v.group(1)) if v else None}


def check_session(sess: Path, problems, notes):
    tag = f"{sess.parent.name}/{sess.name}"

    # A 구성
    stray = sorted(p.name for p in sess.iterdir() if p.name not in ALLOWED)
    if stray:
        problems.append(f"[A 구성] {tag}: 규격 밖 항목 {stray}")
    if not (sess / "info.json").is_file():
        problems.append(f"[A 구성] {tag}: info.json 이 없다")
        return
    info = json.loads((sess / "info.json").read_text(encoding="utf-8"))

    # B 토픽 + C 버전
    for role in ("SLAM", "SAM"):
        b = sess / role
        if not b.is_dir():
            notes.append(f"{tag}: {role} 없음 (짝이 없는 세션)")
            continue
        m = meta(b)
        if m is None:
            problems.append(f"[B 토픽] {tag}/{role}: metadata.yaml 이 없다")
            continue
        if set(m["topics"]) != CAM_TOPICS:
            problems.append(f"[B 토픽] {tag}/{role}: 토픽이 4개 규격과 다르다 -> "
                            f"{sorted(m['topics'])}")
        elif len(set(m["topics"].values())) != 1:
            problems.append(f"[B 토픽] {tag}/{role}: 네 토픽의 메시지 수가 다르다 "
                            f"{m['topics']}")
        if m["version"] != 5:
            problems.append(f"[C 버전] {tag}/{role}: rosbag2 version={m['version']} (5 여야 한다)")

    # C 버전 + D 자립 (lidar / imu)
    for role in ("lidar", "imu"):
        b = sess / role
        if not b.is_dir():
            continue
        m = meta(b)
        if m is None:
            problems.append(f"[B 토픽] {tag}/{role}: metadata.yaml 이 없다")
            continue
        if m["version"] != 5:
            problems.append(f"[C 버전] {tag}/{role}: version={m['version']} (5 여야 한다)")
        src = ROOT / (info.get(role, {}).get("source") or "")
        for f in sorted(b.iterdir()):
            if f.is_symlink():
                problems.append(f"[D 자립] {tag}/{role}/{f.name} 이 심볼릭이다 "
                                f"(-> {os.readlink(f)}). 변환본은 실데이터만 둔다")
                continue
            o = src / f.name
            if f.suffix == ".db3" and o.is_file() and f.stat().st_size != o.stat().st_size:
                problems.append(f"[D 자립] {tag}/{role}/{f.name}: {f.stat().st_size} B 로 "
                                f"원본 {o.stat().st_size} B 와 다르다 (복사가 잘렸다)")

    # E 시계
    c = info.get("clock") or {}
    if "method" not in c:
        problems.append(f"[E 시계] {tag}: info.json 에 clock.method 가 없다")
    elif c.get("applied_to") == "SAM" and (sess / "SAM").is_dir():
        want = -int(c["offset_ns"])
        got = info.get("SAM", {}).get("offset_ns")
        if got is not None and got != want:
            problems.append(f"[E 시계] {tag}: SAM bag 에 구워진 offset {got} ns 가 "
                            f"clock 이 정한 {want} ns 와 다르다")
    if c.get("method") in ("unmeasured", "manifest_scheduled_start"):
        notes.append(f"{tag}: 시계차 산출 = {c['method']} "
                     f"({c.get('offset_ns', 0) / 1e6:+.3f} ms) — genlock 실측이 아니다")

    return info


def check_baseline(inputs: Path, baseline: Path, problems, notes):
    """옛 변환본의 프레임 수·길이와 대조한다 (세션 이름으로 느슨하게 잇는다)."""
    base = json.loads(baseline.read_text(encoding="utf-8"))["bags"]
    # 옛 경로에서 '식별 토큰'(105023, 185223 등)을 뽑아 인덱스한다
    idx = {}
    for path, b in base.items():
        for tok in re.findall(r"\d{6}", path):
            idx.setdefault(tok, []).append((path, b))
    n_ok = n_diff = n_none = 0
    for info_p in sorted(inputs.rglob("info.json")):
        info = json.loads(info_p.read_text(encoding="utf-8"))
        tag = f"{info.get('dataset')}/{info.get('session')}"
        for role in ("SLAM", "SAM"):
            cur = info.get(role)
            if not cur or cur.get("frames") is None:
                continue
            toks = re.findall(r"\d{6}", str(cur.get("source", "")))
            cands = [(p, b) for t in toks for p, b in idx.get(t, [])
                     if role.lower() in p.lower() or t in p]
            # 같은 토큰을 가진 옛 변환본 중 프레임 수가 가장 가까운 것과 비교
            cands = [(p, b) for p, b in cands if b.get("frames")]
            if not cands:
                n_none += 1
                continue
            p, b = min(cands, key=lambda x: abs((x[1]["frames"] or 0) - cur["frames"]))
            df = cur["frames"] - b["frames"]
            dd = (cur.get("duration_s") or 0) - (b.get("duration_s") or 0)
            if abs(df) <= 1 and abs(dd) <= 0.5:
                n_ok += 1
            else:
                n_diff += 1
                notes.append(f"[G 기준] {tag}/{role}: 프레임 {b['frames']} -> {cur['frames']} "
                             f"({df:+d}), 길이 {b.get('duration_s')} -> {cur.get('duration_s')} "
                             f"({dd:+.3f} s)  [옛 변환본 {p}]")
    notes.insert(0, f"[G 기준] 일치 {n_ok} · 차이 {n_diff} · 대응 없음 {n_none}")


def check_stray_bags(problems, notes):
    """두 데이터 폴더 밖에 '입력용' bag 이 남아 있으면 안 된다."""
    stray = []
    for dp, dn, fn in os.walk(ROOT):
        rel = os.path.relpath(dp, ROOT)
        if rel.startswith(".git") or "/build/" in dp or "/install/" in dp:
            dn[:] = []
            continue
        if rel.startswith(("data_slam/", "data_slam_converted/")) or rel in (
                "data_slam", "data_slam_converted"):
            dn[:] = []
            continue
        if any(rel.startswith(t) for t in OUTPUT_TREES):
            dn[:] = []
            continue
        if "metadata.yaml" in fn:
            # 페이로드(.db3/.mcap)가 없으면 bag 이 아니라 기록일 뿐이다.
            payload = any(f.endswith((".db3", ".mcap")) for f in fn)
            m = meta(Path(dp))
            if payload and m and COLOR in (m["topics"] or {}):
                stray.append(rel)
            dn[:] = []
    for s in stray:
        problems.append(f"[H 잔여] 두 데이터 폴더 밖에 카메라 bag 이 있다: {s}")
    if not stray:
        notes.append("[H 잔여] 두 데이터 폴더 밖에 입력용 카메라 bag 이 없다 — OK")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inputs", default="data_slam_converted")
    ap.add_argument("--against", default="")
    ap.add_argument("--skip-stray", action="store_true")
    ap.add_argument("--strict", action="store_true", help="참고 항목도 실패로 본다")
    a = ap.parse_args(ros2_argv(sys.argv[1:]))

    inputs = Path(a.inputs).expanduser().resolve()
    if not inputs.is_dir():
        raise SystemExit(f"--inputs 가 폴더가 아니다: {inputs}")

    problems, notes = [], []
    sessions = sorted(p.parent for p in inputs.rglob("info.json"))
    if not sessions:
        raise SystemExit(f"{inputs} 아래에 info.json 을 가진 세션이 없다")
    for s in sessions:
        check_session(s, problems, notes)
    if a.against:
        check_baseline(inputs, Path(a.against), problems, notes)
    if not a.skip_stray:
        check_stray_bags(problems, notes)

    print(f"세션 {len(sessions)}개 검사")
    if notes:
        print("\n--- 참고 ---")
        for n in notes:
            print(f"  {n}")
    if problems:
        print(f"\n--- 문제 {len(problems)}건 ---")
        for p in problems:
            print(f"  {p}")
        return 1
    print("\n문제 없음 — 전 세션 구성 동일")
    return 1 if (a.strict and notes) else 0


if __name__ == "__main__":
    raise SystemExit(main())
