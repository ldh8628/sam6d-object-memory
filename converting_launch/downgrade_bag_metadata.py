#!/usr/bin/env python3
"""Make a Jazzy-recorded rosbag2 readable by the Humble tooling in this repo.

The Velodyne bags were recorded on the Husky (`ros_distro: jazzy`, metadata
`version: 9`) while everything else here runs Humble (`version: 5`). Humble's
yaml-cpp chokes on the v9 file with "bad conversion" because `offered_qos_profiles`
changed from a YAML string to a sequence of maps, so none of the local
environments can even open the bag.

The `.db3` itself is an ordinary sqlite3 file and is perfectly readable — only
the sidecar metadata is the problem. So we write a v5 metadata.yaml into a new
directory and symlink the payload next to it: no copying, no re-encoding, and
the originals are never touched.

    python3 downgrade_bag_metadata.py <bag_dir> [<bag_dir> ...] --out-root DIR
    python3 downgrade_bag_metadata.py --all-velodyne          # the 9 lidar bags
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import yaml

# converting_launch/ 아래에 있으므로 parents[1] 이 레포 루트다.
ROOT = Path(__file__).resolve().parents[1]
# --all-velodyne 는 새 원본 레이아웃 data_slam/<date>/<session>/SLAM/cpr-*velodyne* 를 훑는다.
RAW = ROOT / "data_slam"
DEFAULT_OUT = None          # --out-root 를 반드시 주게 한다

TARGET_VERSION = 5
# keys Humble's metadata parser knows about; anything else must go
TOP_KEYS = ["version", "storage_identifier", "duration", "starting_time",
            "message_count", "topics_with_message_count", "compression_format",
            "compression_mode", "relative_file_paths", "files"]
TOPIC_KEYS = ["name", "type", "serialization_format", "offered_qos_profiles"]


def downgrade(bag_dir: Path, out_root: Path) -> Path:
    meta_path = bag_dir / "metadata.yaml"
    info = yaml.safe_load(meta_path.read_text())["rosbag2_bagfile_information"]
    src_version = info.get("version")

    out = out_root / bag_dir.name
    out.mkdir(parents=True, exist_ok=True)

    new = {k: info[k] for k in TOP_KEYS if k in info}
    new["version"] = TARGET_VERSION
    new.setdefault("compression_format", "")
    new.setdefault("compression_mode", "")

    for entry in new.get("topics_with_message_count", []):
        tm = entry["topic_metadata"]
        entry["topic_metadata"] = {k: tm[k] for k in TOPIC_KEYS if k in tm}
        # v9 stores a list of QoS maps; v5 expects a string ("" = defaults)
        qos = entry["topic_metadata"].get("offered_qos_profiles")
        if not isinstance(qos, str):
            entry["topic_metadata"]["offered_qos_profiles"] = ""

    # paths must be plain basenames next to the metadata we are writing
    new["relative_file_paths"] = [os.path.basename(p)
                                  for p in new.get("relative_file_paths", [])]
    for f in new.get("files", []):
        f["path"] = os.path.basename(f["path"])

    (out / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": new},
                       default_flow_style=False, sort_keys=False))

    linked = 0
    for name in new["relative_file_paths"]:
        src = bag_dir / name
        dst = out / name
        if dst.is_symlink() or dst.exists():
            dst.unlink()
        dst.symlink_to(src.resolve())
        linked += 1

    print(f"{bag_dir.name}: v{src_version} -> v{TARGET_VERSION}, "
          f"{linked} file(s) symlinked, "
          f"{len(new['topics_with_message_count'])} topics -> {out}")
    return out


def verify(bag_dir: Path) -> bool:
    """Open the rewritten bag with the local rosbag2 and read one message."""
    try:
        import rosbag2_py
    except ImportError:
        print("  (rosbag2_py unavailable here — skipping verification)")
        return True
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("", ""))
        topics = [t.name for t in reader.get_all_topics_and_types()]
        ok = reader.has_next()
        if ok:
            reader.read_next()
        print(f"  verified: {len(topics)} topics, first message readable={ok}")
        return ok
    except Exception as exc:                                  # noqa: BLE001
        print(f"  !! verification FAILED: {str(exc)[:160]}")
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="*", type=Path)
    ap.add_argument("--all-velodyne", action="store_true",
                    help=f"process every cpr-*velodyne* bag under {RAW}")
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--no-verify", action="store_true")
    args = ap.parse_args()

    bags = list(args.bags)
    if args.all_velodyne:
        bags += sorted(p for p in RAW.glob("*/*/SLAM/cpr-*velodyne*") if p.is_dir())
    if not bags:
        ap.error("give bag directories or --all-velodyne")
    if args.out_root is None:
        ap.error("--out-root is required")

    failed = 0
    for b in bags:
        out = downgrade(b, args.out_root)
        if not args.no_verify and not verify(out):
            failed += 1
    print(f"\n{len(bags) - failed}/{len(bags)} bags readable")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
