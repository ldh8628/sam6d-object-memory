#!/usr/bin/env python3
"""Build a shadow-only PEM verification policy sidecar from scored candidates."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROTATION_SWEEP_DEG = (10.0, 15.0, 20.0, 30.0)
TRANSLATION_SWEEP_MM = (25.0, 50.0, 100.0)
TUNE_MIN_ACCEPTED_ACCURACY = 0.90
SYMMETRY_AXES = {
    "Sikhye_high": (0.0, 0.0, 1.0),
    "Sauce_high": (0.0, 0.0, 1.0),
    "Mugcup_high": (0.1057, 0.0337, 0.9938),
}
SYMMETRY_STEP_DEG = 10


class PolicyInputError(ValueError):
    pass


def load_jsonl(path):
    with Path(path).open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise PolicyInputError(f"{path}:{number}: invalid JSON: {exc}") from exc


def checked_threshold(name, value):
    if isinstance(value, bool):
        raise PolicyInputError(f"{name} must be finite and in [0, 1]")
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise PolicyInputError(f"{name} must be finite and in [0, 1]")
    return value


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(value))


def _checked_rotation(value, label):
    if (not isinstance(value, list) or len(value) != 3 or
            any(not isinstance(row, list) or len(row) != 3 for row in value)):
        raise PolicyInputError(f"{label} R must be a finite 3x3 matrix")
    matrix = [[float(item) for item in row] for row in value]
    if not all(math.isfinite(item) for row in matrix for item in row):
        raise PolicyInputError(f"{label} R must be a finite 3x3 matrix")
    gram = _matmul(_transpose(matrix), matrix)
    orthogonal_error = max(abs(gram[row][column] - (1.0 if row == column else 0.0))
                           for row in range(3) for column in range(3))
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))
    if orthogonal_error > 1e-3 or abs(determinant - 1.0) > 1e-3:
        raise PolicyInputError(f"{label} R must be a valid SO(3) matrix")
    return matrix


def _checked_translation(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise PolicyInputError(f"{label} t_mm must contain three finite values")
    translation = [float(item) for item in value]
    if not all(math.isfinite(item) for item in translation):
        raise PolicyInputError(f"{label} t_mm must contain three finite values")
    return translation


def _matmul(left, right):
    return [[sum(left[row][k] * right[k][column] for k in range(3))
             for column in range(3)] for row in range(3)]


def _transpose(matrix):
    return [[matrix[column][row] for column in range(3)] for row in range(3)]


def _axis_rotation(axis, degrees):
    norm = math.sqrt(sum(value * value for value in axis))
    x, y, z = (value / norm for value in axis)
    angle = math.radians(degrees)
    cosine, sine, one_minus = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [cosine + x * x * one_minus,
         x * y * one_minus - z * sine,
         x * z * one_minus + y * sine],
        [y * x * one_minus + z * sine,
         cosine + y * y * one_minus,
         y * z * one_minus - x * sine],
        [z * x * one_minus - y * sine,
         z * y * one_minus + x * sine,
         cosine + z * z * one_minus],
    ]


def symmetry_aware_rotation_distance_deg(first, second, object_name):
    """Return object-symmetry-aware geodesic rotation distance in degrees."""
    first = _checked_rotation(first, "first candidate")
    second = _checked_rotation(second, "second candidate")
    symmetries = [[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]]
    if object_name in SYMMETRY_AXES:
        symmetries = [_axis_rotation(SYMMETRY_AXES[object_name], degrees)
                      for degrees in range(0, 360, SYMMETRY_STEP_DEG)]
    first_t = _transpose(first)
    distances = []
    for symmetry in symmetries:
        relative = _matmul(first_t, _matmul(second, symmetry))
        cosine = max(-1.0, min(1.0, (sum(relative[i][i] for i in range(3)) - 1.0) / 2.0))
        distances.append(math.degrees(math.acos(cosine)))
    return min(distances)


def translation_distance_mm(first, second):
    first = _checked_translation(first, "first candidate")
    second = _checked_translation(second, "second candidate")
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def detection_fingerprint(rows):
    """Bind decisions to the exact detection keys and selected poses in a report."""
    entries = []
    for row in rows:
        try:
            entry = {
                "stamp_ns": int(row["stamp_ns"]),
                "i": int(row["i"]),
                "object": str(row["object"]),
                "R": row["R"],
                "t_mm": row["t_mm"],
                "selected_proposal6000_index": (
                    row.get("verify", {}).get("selected_proposal6000_index")),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise PolicyInputError(f"cannot fingerprint detection row: {exc}") from exc
        entries.append(entry)
    entries.sort(key=lambda item: (item["stamp_ns"], item["i"], item["object"]))
    try:
        payload = json.dumps(entries, sort_keys=True, separators=(",", ":"),
                             ensure_ascii=False).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise PolicyInputError(f"cannot fingerprint detection rows: {exc}") from exc
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _candidate_rank(candidate, fallback_rank):
    del fallback_rank  # rank identity must be explicit; array position is never a substitute.
    rank = candidate.get("rank_geo")
    if not isinstance(rank, int) or isinstance(rank, bool):
        raise PolicyInputError("candidate rank_geo must be a nonnegative integer")
    if rank < 0:
        raise PolicyInputError("candidate rank_geo must be a nonnegative integer")
    return rank


def _merge_candidate_poses(analysis_candidates, pose_row):
    pose_by_proposal = {}
    if isinstance(pose_row, dict):
        for candidate in (pose_row.get("verify", {}).get("cands") or []):
            proposal = candidate.get("proposal6000_index")
            if proposal is not None:
                pose_by_proposal[proposal] = candidate
    merged = []
    seen_ranks = set()
    for fallback_rank, raw_candidate in enumerate(analysis_candidates):
        candidate = dict(raw_candidate)
        rank = _candidate_rank(candidate, fallback_rank)
        if rank in seen_ranks:
            raise PolicyInputError(f"duplicate candidate rank_geo: {rank}")
        seen_ranks.add(rank)
        candidate["rank_geo"] = rank
        pose = pose_by_proposal.get(candidate.get("proposal6000_index"), {})
        candidate.setdefault("R", pose.get("R"))
        candidate.setdefault("t_mm", pose.get("t_mm"))
        merged.append(candidate)
    return sorted(merged, key=lambda candidate: candidate["rank_geo"])


def _fallback_candidate(record, rotation_limit_deg, translation_limit_mm):
    candidates = record["candidates"]
    winner = record["winner"]
    if (winner.get("candidate_valid") is not True or
            winner.get("projection_valid") is not True or
            winner.get("R") is None or winner.get("t_mm") is None):
        return {
            "status": "unavailable", "reason": "top1_pose_unavailable",
            "excluded_cluster_size": 0, "excluded_similar_candidate_count": 0,
            "selected": None,
        }
    excluded = 0
    outside = 0
    score_passed = 0
    pose_unavailable = 0
    for candidate in candidates:
        if (candidate.get("candidate_valid") is not True or
                candidate.get("projection_valid") is not True or
                candidate.get("R") is None or candidate.get("t_mm") is None):
            pose_unavailable += 1
            continue
        rotation_distance = candidate.get("_top1_rotation_distance_deg")
        translation_distance = candidate.get("_top1_translation_distance_mm")
        if rotation_distance is None or translation_distance is None:
            rotation_distance = symmetry_aware_rotation_distance_deg(
                winner["R"], candidate["R"], record["object"])
            translation_distance = translation_distance_mm(
                winner["t_mm"], candidate["t_mm"])
        if (rotation_distance <= rotation_limit_deg and
                translation_distance <= translation_limit_mm):
            excluded += 1
            continue
        outside += 1
        texture = candidate.get("texture_score")
        iou = candidate.get("mask_iou")
        if (not _finite_number(texture) or not _finite_number(iou) or
                texture < record["texture_min"] or iou < record["iou_min"]):
            continue
        score_passed += 1
        if not isinstance(candidate.get("correct"), bool):
            raise PolicyInputError(
                f"fallback candidate GT unavailable at rank_geo={candidate['rank_geo']}")
        selected = {
            "rank_geo": candidate["rank_geo"],
            "index300": candidate.get("index300"),
            "proposal6000_index": candidate.get("proposal6000_index"),
            "geometry_score": candidate.get("geometry_score"),
            "texture_score": float(texture),
            "mask_iou": float(iou),
            "R": candidate["R"],
            "t_mm": candidate["t_mm"],
            "candidate_valid": candidate.get("candidate_valid"),
            "projection_valid": candidate.get("projection_valid"),
            "rotation_distance_deg": rotation_distance,
            "translation_distance_mm": translation_distance,
            "gtOk": candidate.get("correct"),
        }
        return {
            "status": "selected", "reason": "highest_original_geometry_rank_passing_both",
            "excluded_cluster_size": excluded,
            "excluded_similar_candidate_count": max(0, excluded - 1),
            "outside_cluster_count": outside,
            "score_passing_count_before_selection": score_passed,
            "selected": selected,
        }
    reason = ("candidate_pose_unavailable" if outside == 0 and pose_unavailable else
              "no_candidate_pose_outside_cluster" if outside == 0 else
              "no_candidate_passes_texture_and_iou")
    return {
        "status": "none", "reason": reason,
        "excluded_cluster_size": excluded,
        "excluded_similar_candidate_count": max(0, excluded - 1),
        "outside_cluster_count": outside,
        "score_passing_count_before_selection": score_passed, "selected": None,
    }


def _cache_fallback_distances(record):
    winner = record["winner"]
    if winner.get("R") is None or winner.get("t_mm") is None:
        return
    for candidate in record["candidates"]:
        if (candidate.get("candidate_valid") is not True or
                candidate.get("projection_valid") is not True or
                candidate.get("R") is None or candidate.get("t_mm") is None):
            continue
        candidate["_top1_rotation_distance_deg"] = (
            symmetry_aware_rotation_distance_deg(
                winner["R"], candidate["R"], record["object"]))
        candidate["_top1_translation_distance_mm"] = translation_distance_mm(
            winner["t_mm"], candidate["t_mm"])


def _fallback_summary(records, rotation_limit_deg, translation_limit_mm):
    stats = {split: {
        "evaluated": 0, "baseline_accepted": 0, "baseline_accepted_correct": 0,
        "baseline_withheld": 0, "replacement_count": 0,
        "replacement_correct": 0, "correct_to_incorrect": 0,
        "accepted": 0, "accepted_correct": 0,
    } for split in ("tune", "holdout")}
    fallbacks = {}
    for record in records:
        split = record["split"]
        current = stats[split]
        current["evaluated"] += 1
        if not record["withheld"]:
            current["baseline_accepted"] += 1
            current["baseline_accepted_correct"] += int(record["winner"]["correct"])
            current["accepted"] += 1
            current["accepted_correct"] += int(record["winner"]["correct"])
            continue
        current["baseline_withheld"] += 1
        fallback = _fallback_candidate(record, rotation_limit_deg, translation_limit_mm)
        fallbacks[record["decision_key"]] = fallback
        selected = fallback.get("selected")
        if selected is None:
            continue
        current["replacement_count"] += 1
        current["replacement_correct"] += int(selected.get("gtOk") is True)
        current["correct_to_incorrect"] += int(
            record["winner"]["correct"] is True and selected.get("gtOk") is False)
        current["accepted"] += 1
        current["accepted_correct"] += int(selected.get("gtOk") is True)
    for current in stats.values():
        current["accepted_accuracy"] = (
            current["accepted_correct"] / current["accepted"]
            if current["accepted"] else None)
        current["coverage"] = (
            current["accepted"] / current["evaluated"]
            if current["evaluated"] else None)
    return stats, fallbacks


def build_policy(detections, candidates, depth_min, texture_min, iou_min,
                 pose_candidates=None):
    depth_min = checked_threshold("depth-min", depth_min)
    texture_min = checked_threshold("texture-min", texture_min)
    iou_min = checked_threshold("iou-min", iou_min)
    detections = list(detections)
    candidates = list(candidates)
    pose_candidates = detections if pose_candidates is None else list(pose_candidates)
    if len(detections) != len(candidates):
        raise PolicyInputError(
            f"row count mismatch: detections={len(detections)} candidates={len(candidates)}")
    frame_keys = sorted({
        (int(row["stamp_ns"]), int(row["frame_index"]))
        for row in candidates if row.get("reference_role") == "evaluation_frame"
    })
    tune_keys = set(frame_keys[:len(frame_keys) // 2])
    if len(pose_candidates) != len(candidates):
        raise PolicyInputError(
            f"row count mismatch: pose_candidates={len(pose_candidates)} "
            f"candidates={len(candidates)}")
    decisions = {}
    fallback_records = []
    seen_keys = set()
    summary = {split: {"evaluated": 0, "accepted": 0, "withheld": 0,
                       "accepted_correct": 0, "true_reject": 0,
                       "false_reject": 0}
               for split in ("tune", "holdout")}
    for detection, analysis, pose_row in zip(detections, candidates, pose_candidates):
        detection_key = (int(detection["stamp_ns"]), int(detection["i"]),
                         detection["object"])
        analysis_key = (int(analysis["stamp_ns"]), int(analysis["frame_index"]),
                        analysis["object"])
        if detection_key != analysis_key:
            raise PolicyInputError(
                f"row key mismatch: detection={detection_key} candidate={analysis_key}")
        pose_key = (int(pose_row["stamp_ns"]), int(pose_row["i"]), pose_row["object"])
        if pose_key != analysis_key:
            raise PolicyInputError(
                f"row key mismatch: pose={pose_key} candidate={analysis_key}")
        decision_key = f"{analysis_key[0]}:{analysis_key[1]}:{analysis_key[2]}"
        if decision_key in seen_keys:
            raise PolicyInputError(f"duplicate source key: {decision_key}")
        seen_keys.add(decision_key)
        if (analysis.get("reference_role") != "evaluation_frame" or
                analysis.get("stage6000", {}).get("gt_status") != "available"):
            continue
        depth = detection.get("pem", {}).get("depth_valid_frac")
        if (not isinstance(depth, (int, float)) or isinstance(depth, bool) or
                not math.isfinite(depth)):
            raise PolicyInputError(f"invalid depth_valid_frac at {analysis_key}")
        if depth < depth_min:
            continue
        stage_candidates = _merge_candidate_poses(
            analysis["stage300"]["candidates"], pose_row)
        selected = [candidate for candidate in stage_candidates
                    if candidate.get("geometry_selected") is True]
        if len(selected) != 1:
            raise PolicyInputError(f"expected one geometry winner at {analysis_key}")
        winner = selected[0]
        reported_proposal = detection.get("verify", {}).get(
            "selected_proposal6000_index")
        if (reported_proposal is not None and
                winner.get("proposal6000_index") != reported_proposal):
            raise PolicyInputError(f"analysis winner proposal mismatch at {analysis_key}")
        texture = winner.get("texture_score")
        iou = winner.get("mask_iou")
        if not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                   and math.isfinite(value)
                   for value in (texture, iou)):
            raise PolicyInputError(f"nonfinite winner score at {analysis_key}")
        gt_ok = winner.get("correct")
        if not isinstance(gt_ok, bool):
            raise PolicyInputError(f"winner correctness unavailable at {analysis_key}")
        split = "tune" if analysis_key[:2] in tune_keys else "holdout"
        withheld = texture < texture_min and iou < iou_min
        status = "withheld" if withheld else "accepted"
        decision = {
            "status": status,
            "reason": ("texture_and_iou_below_threshold" if withheld
                       else "independent_support_present"),
            "depth": float(depth),
            "texture": float(texture),
            "iou": float(iou),
            "textureFailed": bool(texture < texture_min),
            "iouFailed": bool(iou < iou_min),
            "evaluationEligible": True,
            "split": split,
            "gtOk": gt_ok,
        }
        decisions[decision_key] = decision
        fallback_records.append({
            "decision_key": decision_key, "object": analysis_key[2], "split": split,
            "winner": winner, "candidates": stage_candidates, "withheld": withheld,
            "texture_min": texture_min, "iou_min": iou_min,
        })
        stats = summary[split]
        stats["evaluated"] += 1
        stats[status] += 1
        if status == "accepted" and gt_ok:
            stats["accepted_correct"] += 1
        if status == "withheld":
            stats["false_reject" if gt_ok else "true_reject"] += 1
    for stats in summary.values():
        stats["accepted_accuracy"] = (
            stats["accepted_correct"] / stats["accepted"] if stats["accepted"] else None)
        stats["coverage"] = (
            stats["accepted"] / stats["evaluated"] if stats["evaluated"] else None)

    for record in fallback_records:
        if record["withheld"]:
            _cache_fallback_distances(record)

    sweep = []
    selected_thresholds = None
    selected_stats = None
    best_selection_key = None
    tune_records = [record for record in fallback_records if record["split"] == "tune"]
    for rotation_limit in ROTATION_SWEEP_DEG:
        for translation_limit in TRANSLATION_SWEEP_MM:
            fallback_stats, _ = _fallback_summary(
                tune_records, rotation_limit, translation_limit)
            tune = fallback_stats["tune"]
            eligible = (tune["accepted_accuracy"] is not None and
                        tune["accepted_accuracy"] >= TUNE_MIN_ACCEPTED_ACCURACY and
                        tune["correct_to_incorrect"] == 0)
            row = {
                "rotation_threshold_deg": rotation_limit,
                "translation_threshold_mm": translation_limit,
                "eligible": eligible,
                "tune": tune,
            }
            sweep.append(row)
            selection_key = ((tune["coverage"], -translation_limit, rotation_limit)
                             if eligible else None)
            if selection_key is not None and (
                    best_selection_key is None or selection_key > best_selection_key):
                best_selection_key = selection_key
                selected_thresholds = {
                    "rotation_threshold_deg": rotation_limit,
                    "translation_threshold_mm": translation_limit,
                }
    fallback_enabled = selected_thresholds is not None
    if fallback_enabled:
        selected_stats, selected_fallbacks = _fallback_summary(
            fallback_records,
            selected_thresholds["rotation_threshold_deg"],
            selected_thresholds["translation_threshold_mm"])
        for decision_key, fallback in selected_fallbacks.items():
            decisions[decision_key]["fallbackShadow"] = fallback
        for row in sweep:
            row["selected"] = all(
                row[key] == selected_thresholds[key] for key in selected_thresholds)
    return {
        "schema_version": 1,
        "enabled": True,
        "mode": "shadow_only",
        "selection_method": "geometry_only_unchanged",
        "depth_valid_frac_min": depth_min,
        "texture_score_min": texture_min,
        "mask_iou_min": iou_min,
        "reject_when": "texture_score_below_min_AND_mask_iou_below_min",
        "scope_note": "GT-available evaluation detections; time-blocked half split.",
        "decision_source": "full stage300 score analysis JSONL",
        "report_fingerprint": detection_fingerprint(detections),
        "summary": summary,
        "fallback_shadow": {
            "enabled": fallback_enabled,
            "mode": "shadow_only",
            "selection_method": "original_geometry_rank_after_pose_cluster",
            "candidate_scope": "full_stage300",
            "apply_when": "top1_withheld_by_texture_AND_mask_iou",
            "candidate_pass_when": "texture_score_at_or_above_min_AND_mask_iou_at_or_above_min",
            "pose_cluster_when": "symmetry_aware_rotation_at_or_below_threshold_AND_translation_at_or_below_threshold",
            "actual_pem_pose": "unchanged",
            "tune_constraints": {
                "accepted_accuracy_min": TUNE_MIN_ACCEPTED_ACCURACY,
                "correct_to_incorrect_max": 0,
                "objective": "maximum_coverage",
                "tie_break": "smaller_translation_threshold_then_larger_rotation_threshold",
            },
            "selected_thresholds": selected_thresholds,
            "tune_sweep": sweep,
            "summary": selected_stats,
            **({"reason": "no_tune_threshold_satisfied_constraints"}
               if not fallback_enabled else {}),
        },
        "decisions": decisions,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--detections", required=True)
    parser.add_argument("--candidates", required=True)
    parser.add_argument("--pose-candidates", default="",
                        help="aligned detection JSONL containing candidate R/t; defaults to detections")
    parser.add_argument("--out", required=True)
    parser.add_argument("--depth-min", type=float, required=True)
    parser.add_argument("--texture-min", type=float, required=True)
    parser.add_argument("--iou-min", type=float, required=True)
    parser.add_argument("--dataset-id", default="")
    args = parser.parse_args()
    try:
        depth = checked_threshold("depth-min", args.depth_min)
        texture = checked_threshold("texture-min", args.texture_min)
        iou = checked_threshold("iou-min", args.iou_min)
        detection_rows = list(load_jsonl(args.detections))
        pose_rows = (list(load_jsonl(args.pose_candidates))
                     if args.pose_candidates else detection_rows)
        policy = build_policy(detection_rows, load_jsonl(args.candidates),
                              depth, texture, iou, pose_rows)
        if args.dataset_id:
            policy["dataset_id"] = args.dataset_id
        output = Path(args.out)
        output.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(policy, ensure_ascii=False, separators=(",", ":"))
        output.write_text("window.PEM_VERIFICATION_POLICY=" + payload + ";\n",
                          encoding="utf-8")
    except (OSError, PolicyInputError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps({"out": str(output), "summary": policy["summary"],
                      "decisions": len(policy["decisions"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
