"""Map SAM-6D frame indices to timestamps using the shared ros2 bag.

This is the *only* use of the ros2 bag in the pipeline: read the color image
topic's messages in recorded order and return the list of timestamps. The Nth
color message corresponds to SAM-6D frame index N, so:

    frame_timestamp(frame_idx) = timestamps[frame_idx]

Read-only; never modifies or copies the bag.
"""

from __future__ import annotations

import glob
import os
import sqlite3
from typing import List, Optional


def _resolve_db3(bag_path: str) -> str:
    bag_path = os.path.expanduser(bag_path)
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return bag_path
    candidates = []
    candidates.extend(glob.glob(os.path.join(bag_path, "*.db3")))
    candidates.extend(glob.glob(os.path.join(bag_path, "bag", "*.db3")))
    candidates.extend(glob.glob(os.path.join(bag_path, "*", "*.db3")))
    candidates = sorted(set(candidates))
    if not candidates:
        raise FileNotFoundError(f"no .db3 found under: {bag_path}")
    return candidates[0]


def _find_color_topic(cur, color_topic_hint: str) -> tuple:
    cur.execute("select id, name from topics")
    rows = cur.fetchall()
    # exact hint match first, then substring, then generic color/rgb image.
    for topic_id, name in rows:
        if name == color_topic_hint:
            return topic_id, name
    for topic_id, name in rows:
        if color_topic_hint and color_topic_hint in name:
            return topic_id, name
    for topic_id, name in rows:
        low = name.lower()
        if ("color" in low or "rgb" in low) and "image" in low and "camera_info" not in low:
            return topic_id, name
    raise ValueError(
        f"no color image topic found (hint={color_topic_hint!r}); "
        f"topics={[n for _, n in rows]}"
    )


def load_frame_timestamps(
    bag_path: str,
    color_topic_hint: str = "/camera/camera/color/image_raw",
    scale_to_seconds: bool = True,
) -> List[float]:
    """Return color-image timestamps in recorded order (index == frame index)."""
    db3 = _resolve_db3(bag_path)
    conn = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    try:
        cur = conn.cursor()
        topic_id, _name = _find_color_topic(cur, color_topic_hint)
        cur.execute(
            "select timestamp from messages where topic_id = ? order by timestamp",
            (topic_id,),
        )
        stamps = [row[0] for row in cur.fetchall()]
    finally:
        conn.close()
    if scale_to_seconds:
        # ros2 bag stores nanoseconds.
        return [s / 1e9 for s in stamps]
    return [float(s) for s in stamps]


def frame_timestamp(timestamps: List[float], frame_idx: int) -> Optional[float]:
    if 0 <= frame_idx < len(timestamps):
        return timestamps[frame_idx]
    return None
