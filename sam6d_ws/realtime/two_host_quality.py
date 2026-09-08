"""Measure existing ObjectMemory projections against the processed RGB-D frame."""
import numpy as np

from slam_pose_memory import ObjectAnchorManager, valid_se3


def measure_quality(snapshot, context, model_points_m, masks, depth, K):
    result = {"samples": [], "passed": None, "reason": None}
    if not snapshot or snapshot.get("map_id") != context.get("map_id"):
        return dict(result, reason="landmark_snapshot_missing_or_wrong_map")
    twc = np.asarray(context.get("T_map_camera"), float)
    if context.get("tracking_state") != "TRACKING_OK" or not valid_se3(twc):
        return dict(result, reason="frame_pose_unavailable")
    manager = ObjectAnchorManager(context["map_id"])
    manager.anchors = {name: np.asarray(pose, float)
                       for name, pose in snapshot.get("anchors", {}).items()
                       if valid_se3(np.asarray(pose, float))}
    missing_models = []
    for name in manager.anchors:
        if name not in model_points_m:
            missing_models.append(name)
            continue
        _, diag = manager.output_decision(name, twc, model_points_m[name], K,
                                          depth.shape, masks.get(name), depth,
                                          mode="ism_associated")
        if diag.get("fov_fraction", 0) < manager.config["fov_fraction_min"]:
            continue
        result["samples"].append({"object": name,
            "visible_fraction": diag["fov_fraction"],
            "mask_overlap": diag.get("anchor_ism_overlap"),
            "depth_residual_mm": diag.get("anchor_depth_delta_mm"),
            "passed": bool(diag["output"]), "reason": diag.get("reason")})
    result["missing_models"] = missing_models
    if missing_models:
        return dict(result, passed=None, reason="landmark_models_missing")
    if result["samples"]:
        result["passed"] = all(sample["passed"] for sample in result["samples"])
    else:
        result["reason"] = "no_visible_registered_landmarks"
    return result
