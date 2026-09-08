"""Real-time (streaming) object_memory driver.

Unlike the offline batch runner (which holds ALL poses up front and may look at
a future sample), this feeds events in TIMESTAMP / ARRIVAL order into a
PoseBuffer + StreamingObjectMemory, exactly as a live system would:

  - each SLAM pose is added to the buffer at its own stamp (fast, low latency);
  - each SAM-6D detection batch (one keyframe) ARRIVES LATE -- at capture_stamp
    + latency -- and is fused against the buffered camera pose at its CAPTURE
    stamp (time alignment). A batch whose capture stamp has no pose within the
    buffer (tracking gap / too old) is dropped (INV-008).

This is the code that Phase B (live ROS node) reuses verbatim: only the event
source changes (file replay here -> live subscriber callbacks there). The offline
batch result is the ground truth this driver is validated against (latency->0 and
interpolation off must reproduce it).
"""

from __future__ import annotations

import os
import sys
from typing import Callable, Dict, List, Optional

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "src")))

from core.pose_buffer import PoseBuffer                              # noqa: E402
from core.models import SlamCameraPose                              # noqa: E402
from pipeline.object_memory_runner import (                        # noqa: E402
    FrameResult,
    RunResult,
    StreamingObjectMemory,
)


def _frame_stamp(frame_idx: int, dets, timestamps) -> Optional[float]:
    """Capture stamp of a SAM keyframe (same rule as the offline runner)."""
    if 0 <= frame_idx < len(timestamps):
        return timestamps[frame_idx]
    if dets:
        return dets[0].stamp
    return None


def run_streaming(
    slam_poses: List[SlamCameraPose],
    detections_by_frame: Dict[int, list],
    timestamps: List[float],
    latency: float = 0.15,
    latency_fn: Optional[Callable[[int, list], float]] = None,
    time_align: bool = True,
    interpolate: bool = True,
    buffer_window_s: float = 10.0,
    pose_tol: float = 0.05,
    **mem_kwargs,
) -> RunResult:
    """Replay poses + detections as an asynchronous stream.

    latency / latency_fn : how long a keyframe's SAM result takes to arrive
        (constant, or per-keyframe if latency_fn given). The batch is fused at
        wall-time = capture + latency.
    time_align : if True, fuse against the pose at the CAPTURE stamp (correct);
        if False, fuse against the pose at ARRIVAL time (the buggy 'no time
        alignment' baseline -- demonstrates landmark smearing under motion).
    interpolate : PoseBuffer interpolation between bracketing samples.
    mem_kwargs  : forwarded to StreamingObjectMemory (gates, pd_base, weights,
        quality_weighting, cam_K, img_size, ...).
    """
    buf = PoseBuffer(window_s=buffer_window_s, tol=pose_tol,
                     interpolate=interpolate)
    mem = StreamingObjectMemory(**mem_kwargs)

    # Build the event stream. type priority: pose (0) before sam (1) at equal
    # wall time, so a pose stamped exactly at a batch's arrival is buffered first.
    events = []  # (wall_time, type_priority, payload)
    for p in slam_poses:
        events.append((p.stamp, 0, ("pose", p)))
    for frame_idx in detections_by_frame:
        dets = detections_by_frame[frame_idx]
        capture = _frame_stamp(frame_idx, dets, timestamps)
        if capture is None:
            continue
        lat = latency_fn(frame_idx, dets) if latency_fn is not None else latency
        arrival = capture + lat
        events.append((arrival, 1, ("sam", frame_idx, capture, dets)))
    events.sort(key=lambda e: (e[0], e[1]))

    frame_results: List[FrameResult] = []
    frames_with_slam = 0
    frames_without_slam = 0

    for wall_time, _prio, payload in events:
        if payload[0] == "pose":
            buf.add(payload[1])
            continue
        _, frame_idx, capture, dets = payload
        lookup_stamp = capture if time_align else wall_time
        pose = buf.lookup(lookup_stamp)
        if pose is None:
            frames_without_slam += 1
            fr = mem.step_no_pose(dets, capture, frame_idx)
        else:
            frames_with_slam += 1
            # lifecycle timestamps always use the CAPTURE stamp (when the frame
            # was actually seen), regardless of arrival, so decay/age stay honest.
            fr = mem.step(dets, pose, capture, frame_idx)
        frame_results.append(fr)

    # order frame_results by keyframe for stable reporting/diffing
    frame_results.sort(key=lambda fr: fr.frame_idx)
    return RunResult(
        store=mem.store,
        frame_results=frame_results,
        live_ids_by_name=mem.live_ids_by_name,
        object_ids_by_name=mem.object_ids_by_name(),
        frames_total=len(detections_by_frame),
        frames_with_slam=frames_with_slam,
        frames_without_slam=frames_without_slam,
        detections_total=mem.detections_total,
    )
