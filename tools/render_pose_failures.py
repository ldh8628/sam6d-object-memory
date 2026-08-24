#!/usr/bin/env python3
"""Extract and annotate representative pseudo-GT and failed-pose frames."""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


COLOR_TOPIC = "/camera/camera/color/image_raw"


def read_jsonl(path):
    with Path(path).open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def select_cases(failures, method, per_object):
    groups = defaultdict(list)
    for row in failures:
        if method and row["method"] != method:
            continue
        groups[row["object"]].append(row)
    selected = []
    for obj, rows in sorted(groups.items()):
        rows.sort(key=lambda r: (-float(r.get("rotation_error_deg", 0.0)),
                                 -float(r.get("translation_error_m", 0.0))))
        # Spread examples over time instead of returning adjacent near-duplicates.
        picked = []
        for row in rows:
            if all(abs(row["stamp_ns"] - p["stamp_ns"]) > 500_000_000 for p in picked):
                picked.append(row)
            if len(picked) >= per_object:
                break
        selected.extend(picked)
    return selected


def decode_image(message):
    arr = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, -1)
    if message.encoding.lower() == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(arr[:, :, :3])


def annotate(image, row, color):
    canvas = image.copy()
    if row.get("bbox"):
        x1, y1, x2, y2 = [int(v) for v in row["bbox"]]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 3)
    title = f"{row['object']}  frame={row.get('frame_index', -1)}"
    if "rotation_error_deg" in row:
        title += (f"  rot={row['rotation_error_deg']:.1f}deg"
                  f"  trans={100*row['translation_error_m']:.1f}cm")
    else:
        title += "  pseudo-GT reference"
    subtitle = f"method={row.get('method', 'reference')} score={row.get('score', 0):.3f}"
    verdict = (row.get("verify") or {}).get("verdict")
    if verdict:
        subtitle += f" verify={verdict}"
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 58), (0, 0, 0), -1)
    cv2.putText(canvas, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1,
                cv2.LINE_AA)
    cv2.putText(canvas, subtitle, (8, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1,
                cv2.LINE_AA)
    return canvas


def sheet(images, columns=3):
    if not images:
        return None
    h, w = images[0].shape[:2]
    rows = (len(images) + columns - 1) // columns
    result = np.full((rows * h, columns * w, 3), 32, np.uint8)
    for i, image in enumerate(images):
        result[(i // columns) * h:(i // columns + 1) * h,
               (i % columns) * w:(i % columns + 1) * w] = image
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--failures", required=True)
    parser.add_argument("--pseudo-gt", required=True)
    parser.add_argument("--method", default="geometry_texture")
    parser.add_argument("--per-object", type=int, default=6)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    failures = select_cases(read_jsonl(args.failures), args.method, args.per_object)
    pseudo_gt = json.loads(Path(args.pseudo_gt).read_text(encoding="utf-8"))
    references = []
    for obj, gt in pseudo_gt.items():
        for row in gt.get("representative_frames", [])[:3]:
            references.append({"object": obj, **row})
    wanted = {int(r["stamp_ns"]): r for r in failures + references}

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    found = {}
    while reader.has_next() and len(found) < len(wanted):
        topic, data, _ = reader.read_next()
        if topic != COLOR_TOPIC:
            continue
        message = deserialize_message(data, Image)
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if stamp in wanted:
            found[stamp] = decode_image(message)

    out = Path(args.out)
    (out / "individual").mkdir(parents=True, exist_ok=True)
    grouped_fail, grouped_ref = defaultdict(list), defaultdict(list)
    for kind, rows, color, groups in (("failure", failures, (0, 0, 255), grouped_fail),
                                      ("reference", references, (0, 220, 0), grouped_ref)):
        for row in rows:
            image = found.get(int(row["stamp_ns"]))
            if image is None:
                continue
            rendered = annotate(image, row, color)
            groups[row["object"]].append(rendered)
            path = out / "individual" / f"{kind}_{row['object']}_{row['stamp_ns']}.jpg"
            cv2.imwrite(str(path), rendered)
    for obj, images in grouped_fail.items():
        cv2.imwrite(str(out / f"failures_{obj}.jpg"), sheet(images))
    for obj, images in grouped_ref.items():
        cv2.imwrite(str(out / f"reference_{obj}.jpg"), sheet(images))
    summary = {"requested": len(wanted), "extracted": len(found),
               "failure_frames": sum(map(len, grouped_fail.values())),
               "reference_frames": sum(map(len, grouped_ref.values()))}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
