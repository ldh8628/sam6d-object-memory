#!/usr/bin/env python3
"""Build reusable PEM candidate score-distribution data and HTML.

The input is the opt-in compact JSONL emitted by ``temp/verify_eval.py``.
Thresholds produced here are proposals only; this module never changes PEM
selection or realtime configuration.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


CHANNELS = {
    "geometry_score": {"label": "PEM geometry (raw)", "direction": "higher_is_better"},
    "geometry_score_detection_normalized": {
        "label": "PEM geometry (per-detection min-max)",
        "direction": "higher_is_better",
    },
    "texture_score": {"label": "Texture comparison", "direction": "higher_is_better"},
    "mask_iou": {"label": "2D mask IoU", "direction": "higher_is_better"},
}
THRESHOLD_CHANNELS = ("geometry_score", "texture_score", "mask_iou")
TEMPLATE_DIR = Path(__file__).resolve().parent / "pem_score_report"


class AnalysisInputError(ValueError):
    pass


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate_compact_distribution(value, where):
    if not isinstance(value, dict):
        raise AnalysisInputError(f"{where} must be an object")
    edges, bins = value.get("edges"), value.get("bins")
    if not isinstance(edges, list) or not isinstance(bins, list) or len(edges) != len(bins) + 1:
        raise AnalysisInputError(f"{where} has invalid histogram dimensions")
    if (any(not _finite(item) for item in edges)
            or any(float(right) <= float(left) for left, right in zip(edges, edges[1:]))
            or any(not isinstance(item, int) or item < 0 for item in bins)):
        raise AnalysisInputError(f"{where} has invalid histogram values")
    count = value.get("count")
    overflow = value.get("overflow", 0)
    missing = value.get("missing_count", 0)
    if (not isinstance(count, int) or not isinstance(overflow, int)
            or not isinstance(missing, int) or min(count, overflow, missing) < 0
            or count != sum(bins) + overflow):
        raise AnalysisInputError(f"{where} violates count=sum(bins)+overflow")


def _validate_correctness(value, where):
    if not isinstance(value, dict):
        raise AnalysisInputError(f"{where} correctness metadata is missing")
    rotation = value.get("rotation_threshold_deg")
    translation = value.get("translation_threshold_mm")
    step = value.get("symmetry_step_deg")
    axis = value.get("symmetry_axis")
    if (not _finite(rotation) or float(rotation) < 0
            or not _finite(translation) or float(translation) < 0
            or isinstance(step, bool) or not isinstance(step, int) or step <= 0
            or (axis is not None and (not isinstance(axis, list) or len(axis) != 3
                                      or any(not _finite(item) for item in axis)
                                      or not math.isclose(sum(float(item) ** 2 for item in axis),
                                                          1.0, rel_tol=0.0, abs_tol=2e-2)))):
        raise AnalysisInputError(f"{where} has invalid correctness metadata")


def load_jsonl(path, expected_candidate_count=300):
    rows = []
    detection_keys = set()
    stamp_to_frame = {}
    frame_to_stamp = {}
    with Path(path).open(encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise AnalysisInputError(f"invalid JSONL row {number}: {exc}") from exc
            stage300 = row.get("stage300") if isinstance(row, dict) else None
            stage6000 = row.get("stage6000") if isinstance(row, dict) else None
            candidates = stage300.get("candidates") if isinstance(stage300, dict) else None
            if (not isinstance(row, dict) or row.get("schema_version") != 1
                    or row.get("reference_role") not in {"reference_member", "evaluation_frame"}
                    or not isinstance(row.get("object"), str)
                    or not isinstance(row.get("stage300"), dict)
                    or not isinstance(candidates, list)
                    or stage300.get("selection_method") != "geometry_only"
                    or not isinstance(stage6000, dict)
                    or stage6000.get("candidate_count") != 6000
                    or stage6000.get("gt_status") not in {"available", "unavailable"}
                    or stage6000.get("units") != "normalized_object_radius"):
                raise AnalysisInputError(f"invalid score-analysis row {number}")
            if (isinstance(row.get("stamp_ns"), bool) or not isinstance(row.get("stamp_ns"), int)
                    or isinstance(row.get("frame_index"), bool)
                    or not isinstance(row.get("frame_index"), int)):
                raise AnalysisInputError(f"row {number} has noninteger detection key")
            detection_key = (row["stamp_ns"], row["frame_index"], row["object"])
            if detection_key in detection_keys:
                raise AnalysisInputError(f"duplicate detection row {number}: {detection_key}")
            detection_keys.add(detection_key)
            previous_frame = stamp_to_frame.setdefault(row["stamp_ns"], row["frame_index"])
            previous_stamp = frame_to_stamp.setdefault(row["frame_index"], row["stamp_ns"])
            if previous_frame != row["frame_index"] or previous_stamp != row["stamp_ns"]:
                raise AnalysisInputError(
                    f"row {number} violates stamp/frame one-to-one mapping")
            candidate_count = stage300.get("candidate_count")
            if (candidate_count != len(candidates)
                    or (expected_candidate_count is not None
                        and candidate_count != int(expected_candidate_count))):
                raise AnalysisInputError(f"row {number} is not a complete candidate population")
            _validate_correctness(stage300.get("correctness"), f"row {number} stage300")
            _validate_correctness(stage6000.get("correctness"), f"row {number} stage6000")
            for label in ("overall", "correct", "incorrect", "invalid_pose"):
                value = stage6000.get(label)
                if value is not None:
                    _validate_compact_distribution(value, f"row {number} stage6000.{label}")
            overall = stage6000["overall"]
            if overall["count"] + overall["missing_count"] != 6000:
                raise AnalysisInputError(f"row {number} stage6000 overall population is incomplete")
            if stage6000.get("gt_status") == "available":
                partitions = [stage6000.get(label) for label in
                              ("correct", "incorrect", "invalid_pose")]
                if any(not isinstance(value, dict) for value in partitions) or sum(
                        value["count"] + value["missing_count"] for value in partitions) != 6000:
                    raise AnalysisInputError(f"row {number} stage6000 labels do not partition 6000")
                if (any(value["edges"] != overall["edges"] for value in partitions)
                        or any(sum(value["bins"][index] for value in partitions)
                               != overall["bins"][index]
                               for index in range(len(overall["bins"])))
                        or sum(value.get("overflow", 0) for value in partitions)
                        != overall.get("overflow", 0)
                        or sum(value.get("missing_count", 0) for value in partitions)
                        != overall.get("missing_count", 0)):
                    raise AnalysisInputError(
                        f"row {number} stage6000 labelled histograms do not match overall")
            elif (any(stage6000.get(label) is not None
                      for label in ("correct", "incorrect", "invalid_pose"))
                  or any(candidate.get("gt_status") != "unavailable"
                         or candidate.get("correct") is not None
                         for candidate in candidates)):
                raise AnalysisInputError(
                    f"row {number} GT-unavailable population contains labels")
            indices = [candidate.get("index300") for candidate in candidates
                       if isinstance(candidate, dict)]
            if (len(indices) != candidate_count or any(
                    isinstance(index, bool) or not isinstance(index, int) for index in indices)
                    or set(indices) != set(range(candidate_count))):
                raise AnalysisInputError(f"row {number} has invalid candidate indices")
            proposal_indices = [candidate.get("proposal6000_index") for candidate in candidates]
            if (any(isinstance(index, bool) or not isinstance(index, int)
                    or index < 0 or index >= 6000 for index in proposal_indices)
                    or len(set(proposal_indices)) != candidate_count):
                raise AnalysisInputError(f"row {number} has invalid proposal6000 indices")
            selected = [candidate for candidate in candidates
                        if candidate.get("geometry_selected") is True]
            if (len(selected) != 1 or selected[0].get("index300") != stage300.get("selected_index300")):
                raise AnalysisInputError(f"row {number} has inconsistent geometry selection")
            for index, candidate in enumerate(candidates):
                status = candidate.get("gt_status") if isinstance(candidate, dict) else None
                correct = candidate.get("correct") if isinstance(candidate, dict) else None
                candidate_valid = candidate.get("candidate_valid", True)
                if (status not in {"available", "unavailable"}
                        or not isinstance(candidate_valid, bool)
                        or (status == "available" and candidate_valid
                            and not isinstance(correct, bool))
                        or (status == "available" and not candidate_valid and correct is not None)
                        or (status == "unavailable" and correct is not None)):
                    raise AnalysisInputError(f"row {number} candidate {index} has invalid GT label")
                for channel in CHANNELS:
                    if candidate.get(channel) is not None and not _finite(candidate[channel]):
                        raise AnalysisInputError(
                            f"row {number} candidate {index} has non-finite {channel}")
            geometry_candidates = [candidate for candidate in candidates
                                   if _finite(candidate.get("geometry_score"))]
            if not geometry_candidates:
                raise AnalysisInputError(f"row {number} has no finite geometry score")
            geometry_winner = max(
                geometry_candidates, key=lambda candidate: candidate["geometry_score"])
            maximum_geometry = geometry_winner["geometry_score"]
            first_original_maximum = min(
                candidate["index300"] for candidate in geometry_candidates
                if candidate["geometry_score"] == maximum_geometry)
            if (selected[0]["index300"] != first_original_maximum
                    or candidates[0] is not selected[0]):
                raise AnalysisInputError(
                    f"row {number} selected candidate is not first geometry argmax")
            rows.append(row)
    if not rows:
        raise AnalysisInputError("score-analysis input is empty")
    return rows


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def _histogram_edges(values, bins=30):
    finite = np.asarray([float(value) for value in values if _finite(value)], dtype=float)
    if not len(finite):
        return None
    lo, hi = float(finite.min()), float(finite.max())
    if math.isclose(lo, hi):
        width = max(abs(lo) * 0.01, 1e-9)
        return np.asarray([lo - width, hi + width])
    return np.linspace(lo, hi, min(int(bins), len(finite)) + 1)


def distribution(values, total_count=None, bins=30, edges=None):
    finite = np.asarray([float(v) for v in values if _finite(v)], dtype=float)
    total = len(values) if total_count is None else int(total_count)
    if not len(finite):
        return {"count": 0, "missing_count": total, "histogram": [],
                "quantiles": {}, "ecdf": []}
    edges = _histogram_edges(finite, bins) if edges is None else np.asarray(edges, dtype=float)
    counts, edges = np.histogram(finite, bins=edges)
    ordered = np.sort(finite)
    take = np.unique(np.linspace(0, len(ordered) - 1,
                                 min(101, len(ordered))).round().astype(int))
    return {
        "count": int(len(finite)), "missing_count": int(max(0, total - len(finite))),
        "histogram": [{"lo": float(edges[i]), "hi": float(edges[i + 1]),
                       "count": int(counts[i])} for i in range(len(counts))],
        "quantiles": {key: float(value) for key, value in zip(
            ("min", "p05", "p25", "p50", "p75", "p95", "max"),
            np.quantile(finite, [0, .05, .25, .5, .75, .95, 1]))},
        "ecdf": [{"score": float(ordered[i]), "fraction": float((i + 1) / len(ordered))}
                 for i in take],
    }


def blocked_split(rows):
    """Preserve time order: first half tune, second half holdout."""
    ordered = sorted(rows, key=lambda row: (int(row["stamp_ns"]), int(row["frame_index"])))
    middle = len(ordered) // 2
    return ordered[:middle], ordered[middle:]


def _candidate_rows(rows):
    for row in rows:
        # A frame can contain several objects, each with an independent set of
        # 300 candidates. Keep those detections separate so the overall
        # surviving-candidate mean can never exceed the per-detection total.
        frame_key = f'{row["stamp_ns"]}:{row["frame_index"]}:{row["object"]}'
        for candidate in row["stage300"]["candidates"]:
            yield frame_key, candidate


def threshold_metrics(rows, channel, threshold):
    by_frame = defaultdict(list)
    correct_total = correct_kept = incorrect_total = incorrect_rejected = 0
    missing = 0
    gt_unavailable = 0
    invalid_pose = 0
    for frame, candidate in _candidate_rows(rows):
        if candidate.get("gt_status") != "available":
            gt_unavailable += 1
            continue
        if not isinstance(candidate.get("correct"), bool):
            invalid_pose += 1
            continue
        correct = candidate["correct"]
        if correct:
            correct_total += 1
        else:
            incorrect_total += 1
        value = candidate.get(channel)
        if not _finite(value):
            missing += 1
            by_frame[frame].append((correct, False))
            incorrect_rejected += int(not correct)
            continue
        keep = float(value) >= float(threshold)
        by_frame[frame].append((correct, keep))
        correct_kept += int(correct and keep)
        incorrect_rejected += int((not correct) and (not keep))
    eligible_frames = [items for items in by_frame.values()
                       if any(correct for correct, _keep in items)]
    frame_kept = sum(any(correct and keep for correct, keep in items)
                     for items in eligible_frames)
    survivor_counts = [sum(keep for _correct, keep in items) for items in by_frame.values()]
    return {
        "correct_candidate_count": correct_total,
        "correct_candidate_retention": (None if not correct_total else
                                        correct_kept / correct_total),
        "incorrect_candidate_count": incorrect_total,
        "incorrect_candidate_rejection": (None if not incorrect_total else
                                           incorrect_rejected / incorrect_total),
        "eligible_detection_count": len(eligible_frames),
        "correct_detections_kept": frame_kept,
        "detection_level_recall": (None if not eligible_frames else
                                    frame_kept / len(eligible_frames)),
        "surviving_candidates_mean": (None if not survivor_counts else
                                      float(np.mean(survivor_counts))),
        "surviving_candidates_median": (None if not survivor_counts else
                                        float(np.median(survivor_counts))),
        "missing_score_count": missing,
        "gt_unavailable_candidate_count": gt_unavailable,
        "invalid_pose_candidate_count": invalid_pose,
    }


def choose_threshold(tune_rows, holdout_rows, channel, minimum_retention=.95,
                     min_correct=20):
    if isinstance(min_correct, bool) or int(min_correct) != min_correct or min_correct < 1:
        raise AnalysisInputError("min_correct must be a positive integer detection count")
    if not _finite(minimum_retention) or not 0.0 < float(minimum_retention) <= 1.0:
        raise AnalysisInputError("minimum_retention must be finite in (0, 1]")
    tune_candidates = [candidate for _frame, candidate in _candidate_rows(tune_rows)
                       if candidate.get("gt_status") == "available"]
    tune_correct_total = sum(candidate.get("correct") is True
                             for candidate in tune_candidates)
    correct_scores = sorted(float(candidate[channel])
                            for _frame, candidate in _candidate_rows(tune_rows)
                            if candidate.get("gt_status") == "available"
                            and candidate.get("correct") is True
                            and _finite(candidate.get(channel)))
    tune_correct_detections = sum(any(
        candidate.get("gt_status") == "available"
        and candidate.get("correct") is True and _finite(candidate.get(channel))
        for candidate in row["stage300"]["candidates"]) for row in tune_rows)
    holdout_correct = sum(candidate.get("correct") is True
                          for _frame, candidate in _candidate_rows(holdout_rows)
                          if candidate.get("gt_status") == "available")
    holdout_correct_scored = sum(candidate.get("correct") is True
                                 and _finite(candidate.get(channel))
                                 for _frame, candidate in _candidate_rows(holdout_rows)
                                 if candidate.get("gt_status") == "available")
    holdout_correct_detections = sum(any(
        candidate.get("gt_status") == "available"
        and candidate.get("correct") is True and _finite(candidate.get(channel))
        for candidate in row["stage300"]["candidates"]) for row in holdout_rows)
    tune_sample = threshold_metrics(tune_rows, channel, float("-inf"))
    holdout_sample = threshold_metrics(holdout_rows, channel, float("-inf"))
    common = {"tune_correct_with_score": len(correct_scores),
              "tune_correct_detection_count": tune_correct_detections,
              "holdout_correct": int(holdout_correct),
              "holdout_correct_with_score": int(holdout_correct_scored),
              "holdout_correct_detection_count": holdout_correct_detections,
              "tune_sample": tune_sample, "holdout_sample": holdout_sample}
    if tune_correct_detections < int(min_correct):
        return {"status": "insufficient_data", "reason": "tune_correct_detections_too_few",
                **common}
    if holdout_correct_detections < int(min_correct):
        return {"status": "insufficient_data", "reason": "holdout_correct_detections_too_few",
                **common}
    allowed_drop = int(math.floor(
        (1.0 - float(minimum_retention)) * tune_correct_total + 1e-12))
    missing_correct = tune_correct_total - len(correct_scores)
    scored_drop = allowed_drop - missing_correct
    if scored_drop < 0 or not correct_scores:
        return {"status": "insufficient_data",
                "reason": "missing_correct_scores_exceed_retention_budget",
                "tune_correct_total": tune_correct_total,
                **common}
    threshold = correct_scores[min(scored_drop, len(correct_scores) - 1)]
    all_scores = sorted({float(candidate[channel])
                         for _frame, candidate in _candidate_rows(tune_rows)
                         if isinstance(candidate.get("correct"), bool)
                         and candidate.get("gt_status") == "available"
                         and _finite(candidate.get(channel))})
    if len(all_scores) > 101:
        ids = np.unique(np.linspace(0, len(all_scores) - 1, 101).round().astype(int))
        sweep_thresholds = [all_scores[i] for i in ids]
    else:
        sweep_thresholds = all_scores
    sweep_thresholds = sorted(set(sweep_thresholds) | {threshold})
    tune_metrics = threshold_metrics(tune_rows, channel, threshold)
    holdout_metrics = threshold_metrics(holdout_rows, channel, threshold)

    def stage_presence(source_rows):
        available = stage6000 = stage300 = 0
        for row in source_rows:
            stage = row.get("stage6000") or {}
            if stage.get("gt_status") != "available":
                continue
            available += 1
            correct = stage.get("correct") or {}
            stage6000 += int((correct.get("count", 0) + correct.get("missing_count", 0)) > 0)
            stage300 += int(any(candidate.get("correct") is True
                                for candidate in row["stage300"]["candidates"]))
        return available, stage6000, stage300

    available, stage6000, stage300 = stage_presence(holdout_rows)
    return {
        "status": "proposed", "threshold": threshold,
        "direction": "keep_score_greater_than_or_equal",
        "minimum_tune_correct_retention": float(minimum_retention),
        "tune_correct_detection_count": tune_correct_detections,
        "tune_correct_candidate_count": tune_correct_total,
        "holdout_correct_candidate_count": int(holdout_correct),
        "holdout_correct_with_score": int(holdout_correct_scored),
        "tune": tune_metrics,
        "holdout": holdout_metrics,
        "holdout_stage_survival": {
            "population": "holdout GT-available object detections",
            "gt_available": available,
            "stage6000_with_correct": stage6000,
            "stage300_with_correct": stage300,
            "post_filter_with_correct": holdout_metrics["correct_detections_kept"],
            "lost_6000_to_300": stage6000 - stage300,
            "lost_300_to_filter": stage300 - holdout_metrics["correct_detections_kept"],
        },
        "sweep": [{"threshold": value,
                   **threshold_metrics(tune_rows, channel, value)}
                  for value in sweep_thresholds],
        "activation": "proposal_only_not_realtime",
    }


def _channel_distributions(rows, channel, labelled):
    candidates = [candidate for _frame, candidate in _candidate_rows(rows)]
    if not labelled:
        return {"unlabelled": distribution([candidate.get(channel) for candidate in candidates],
                                             total_count=len(candidates))}
    correct = [candidate.get(channel) for candidate in candidates
               if candidate.get("gt_status") == "available"
               and candidate.get("correct") is True]
    incorrect = [candidate.get(channel) for candidate in candidates
                 if candidate.get("gt_status") == "available"
                 and candidate.get("correct") is False]
    shared_edges = _histogram_edges(correct + incorrect)
    correct_dist = distribution(correct, total_count=len(correct), edges=shared_edges)
    incorrect_dist = distribution(incorrect, total_count=len(incorrect), edges=shared_edges)
    overlap_low = max(correct_dist.get("quantiles", {}).get("min", math.inf),
                      incorrect_dist.get("quantiles", {}).get("min", math.inf))
    overlap_high = min(correct_dist.get("quantiles", {}).get("max", -math.inf),
                       incorrect_dist.get("quantiles", {}).get("max", -math.inf))
    overlap = (None if overlap_low > overlap_high or not math.isfinite(overlap_low)
               else {"lo": overlap_low, "hi": overlap_high})
    return {"correct": correct_dist,
            "incorrect": incorrect_dist,
            "overlap_interval": overlap,
            "gt_unavailable_count": sum(
                candidate.get("gt_status") != "available" for candidate in candidates),
            "invalid_pose_count": sum(
                candidate.get("gt_status") == "available"
                and candidate.get("candidate_valid") is False
                for candidate in candidates)}


def _merge_stage6000(rows, labelled):
    """Merge fixed residual histograms without materializing 6,000 raw rows."""
    labels = ("correct", "incorrect", "invalid_pose") if labelled else ("overall",)
    merged = {}
    for label in labels:
        edges = bins = None
        missing = overflow = 0
        for row in rows:
            value = (row.get("stage6000") or {}).get(label)
            if not isinstance(value, dict):
                continue
            row_edges = value.get("edges")
            row_bins = value.get("bins")
            if not isinstance(row_edges, list) or not isinstance(row_bins, list):
                continue
            if edges is None:
                edges = [float(x) for x in row_edges]
                bins = [0] * len(row_bins)
            if [float(x) for x in row_edges] != edges or len(row_bins) != len(bins):
                raise AnalysisInputError("inconsistent stage6000 residual histogram schema")
            bins = [left + int(right) for left, right in zip(bins, row_bins)]
            missing += int(value.get("missing_count", 0))
            overflow += int(value.get("overflow", 0))
        if edges is None:
            merged[label] = {"count": 0, "missing_count": 0, "histogram": [],
                             "quantiles": {}, "ecdf": [], "overflow": 0,
                             "units": "normalized_object_radius"}
            continue
        histogram = [{"lo": edges[i], "hi": edges[i + 1], "count": bins[i]}
                     for i in range(len(bins))]
        # Keep aggregation proportional to bin count, never candidate count.
        # Quantiles are explicitly approximate because the 6,000 raw residuals
        # are intentionally not serialized.
        weighted = [((item["lo"] + item["hi"]) / 2.0, item["count"])
                    for item in histogram if item["count"]]
        count = int(sum(bins) + overflow)
        observed_count = int(sum(bins))

        def approximate_quantile(fraction):
            if not count:
                return None
            target = max(1, int(math.ceil(float(fraction) * count)))
            if target > observed_count:
                return None
            cumulative = 0
            for midpoint, weight in weighted:
                cumulative += weight
                if cumulative >= target:
                    return float(midpoint)
            return None

        cumulative = 0
        ecdf = []
        for midpoint, weight in weighted:
            cumulative += weight
            ecdf.append({"score": float(midpoint), "fraction": cumulative / count})
        exact_mins = [((row.get("stage6000") or {}).get(label) or {}).get(
            "quantiles", {}).get("min") for row in rows]
        exact_maxs = [((row.get("stage6000") or {}).get(label) or {}).get(
            "quantiles", {}).get("max") for row in rows]
        merged[label] = {
            "count": count, "missing_count": missing, "histogram": histogram,
            "overflow": overflow,
            "quantiles": {key: approximate_quantile(fraction)
                          for key, fraction in zip(
                              ("min", "p05", "p25", "p50", "p75", "p95", "max"),
                              (0.0, .05, .25, .5, .75, .95, 1.0))},
            "ecdf": ecdf,
            "right_censored_count": overflow,
            "exact_observed_min": min((x for x in exact_mins if _finite(x)), default=None),
            "exact_observed_max": max((x for x in exact_maxs if _finite(x)), default=None),
            "quantile_method": "fixed-bin midpoint approximation; overflow right-censored",
            "units": "normalized_object_radius",
        }
    return merged


def _stage_attrition(rows):
    stage6000 = stage300 = lost = unavailable = eligible = 0
    for row in rows:
        stage = row.get("stage6000") or {}
        if stage.get("gt_status") not in (None, "available") or not isinstance(
                stage.get("correct"), dict):
            unavailable += 1
            continue
        eligible += 1
        correct_value = stage.get("correct") or {}
        correct6000 = ((correct_value.get("count") or 0)
                       + (correct_value.get("missing_count") or 0))
        correct300 = sum(candidate.get("correct") is True
                         for candidate in row["stage300"]["candidates"])
        present6000, present300 = correct6000 > 0, correct300 > 0
        stage6000 += int(present6000)
        stage300 += int(present300)
        lost += int(present6000 and not present300)
    return {"detections": eligible, "gt_unavailable_detections": unavailable,
            "stage6000_with_correct": stage6000,
            "stage300_with_correct": stage300,
            "lost_6000_to_300": lost,
            "stage6000_score": "initial 3-point residual (not PEM geometry)",
            "stage300_scores": "PEM geometry, texture comparison, 2D mask IoU"}


def analyze(rows, pseudo_gt, dataset_id="longcircle2_sam", min_correct=20):
    if isinstance(min_correct, bool) or int(min_correct) != min_correct or min_correct < 1:
        raise AnalysisInputError("min_correct must be a positive integer detection count")
    objects = {name: value for name, value in pseudo_gt.items()
               if not name.startswith("_") and isinstance(value, dict)}
    reference_members = {
        (name, int(frame["stamp_ns"]))
        for name, value in objects.items() if value.get("trusted") is True
        for frame in value.get("reference_frames", [])
    }
    correctness_values = []
    by_object = defaultdict(list)
    evaluation_keys = sorted({(int(row["stamp_ns"]), int(row["frame_index"]))
                              for row in rows
                              if row.get("reference_role") != "reference_member"})
    tune_keys = set(evaluation_keys[:len(evaluation_keys) // 2])

    def dataset_split(object_rows):
        tune = [row for row in object_rows
                if (int(row["stamp_ns"]), int(row["frame_index"])) in tune_keys]
        holdout = [row for row in object_rows
                   if (int(row["stamp_ns"]), int(row["frame_index"])) not in tune_keys]
        return tune, holdout

    for row in rows:
        if row.get("object") not in objects:
            raise AnalysisInputError(f'unknown object in analysis: {row.get("object")}')
        expected_role = ("reference_member"
                         if (row["object"], int(row["stamp_ns"])) in reference_members
                         else "evaluation_frame")
        if row.get("reference_role") != expected_role:
            raise AnalysisInputError(
                f'reference role mismatch for {row["object"]} at {row["stamp_ns"]}')
        stage_correctness = row["stage300"].get("correctness")
        if stage_correctness != (row.get("stage6000") or {}).get("correctness"):
            raise AnalysisInputError("stage6000/stage300 correctness metadata mismatch")
        correctness_values.append(stage_correctness)
        if row.get("reference_role") == "reference_member":
            continue
        by_object[row["object"]].append(row)

    panels = {}
    trusted_rows = []
    for name in sorted(objects):
        value = objects[name]
        trusted = value.get("trusted") is True
        object_rows = by_object.get(name, [])
        panel = {
            "trusted": trusted, "input_detection_count": len(object_rows),
            "excluded_reference_member_count": sum(
                row.get("object") == name and row.get("reference_role") == "reference_member"
                for row in rows),
            "status": "available" if trusted else "gt_trust_insufficient",
            "unavailable_reason": None if trusted else value.get("unavailable_reason"),
            "stage_attrition": _stage_attrition(object_rows) if trusted else None,
            "stage6000_residual": _merge_stage6000(object_rows, labelled=trusted),
            "channels": {}, "thresholds": {},
        }
        tune, holdout = dataset_split(object_rows)
        panel["split"] = {"policy": "time_ordered_blocked_half",
                          "tune_detection_count": len(tune),
                          "holdout_detection_count": len(holdout)}
        for channel in CHANNELS:
            panel["channels"][channel] = _channel_distributions(
                object_rows, channel, labelled=trusted)
        if trusted:
            trusted_rows.extend(object_rows)
            for channel in THRESHOLD_CHANNELS:
                panel["thresholds"][channel] = choose_threshold(
                    tune, holdout, channel, min_correct=min_correct)
        panels[name] = panel

    tune_all, holdout_all = dataset_split(trusted_rows)
    thresholds = {(value.get("rotation_threshold_deg"),
                   value.get("translation_threshold_mm"))
                  for value in correctness_values}
    steps = {value.get("symmetry_step_deg") for value in correctness_values}
    if len(thresholds) != 1 or len(steps) != 1:
        raise AnalysisInputError("inconsistent correctness thresholds or symmetry step")
    rotation_threshold, translation_threshold = next(iter(thresholds))
    symmetry_policy = {}
    for name in sorted(objects):
        axes = {None if value.get("symmetry_axis") is None
                else tuple(float(item) for item in value["symmetry_axis"])
                for row, value in zip(rows, correctness_values) if row["object"] == name}
        if len(axes) > 1:
            raise AnalysisInputError(f"inconsistent symmetry axis for {name}")
        axis = next(iter(axes)) if axes else None
        symmetry_policy[name] = {"axis": None if axis is None else list(axis),
                                 "step_deg": next(iter(steps))}
    return {
        "schema_version": 1,
        "dataset_id": dataset_id,
        "definitions": {
            "correct_candidate": (f"symmetry-aware rotation <={rotation_threshold:g} deg AND "
                                  f"translation <={translation_threshold:g} mm"),
            "trusted_objects": sorted(name for name, value in objects.items()
                                      if value.get("trusted") is True),
            "untrusted_objects": sorted(name for name, value in objects.items()
                                        if value.get("trusted") is not True),
            "reference_members": "excluded from threshold tuning and evaluation",
            "split": "time-ordered blocked split: first half tune, second half holdout",
            "split_frame_counts": {"tune": len(tune_keys),
                                   "holdout": len(evaluation_keys) - len(tune_keys)},
            "units": {"rotation": "degree", "translation": "millimetre",
                      "stage6000_residual": "normalized_object_radius"},
            "correctness_thresholds": {
                "rotation_threshold_deg": rotation_threshold,
                "translation_threshold_mm": translation_threshold,
            },
            "symmetry_policy": symmetry_policy,
            "selection": "geometry-only 300-to-1; unchanged by analysis",
            "threshold_activation": "proposal-only; not active in realtime",
            "self_slam_warning": "same-sequence self-SLAM pseudo-GT exploration; not independent generalization",
            "channels": CHANNELS,
        },
        "overall": {
            "trusted_evaluation_detection_count": len(trusted_rows),
            "stage_attrition": _stage_attrition(trusted_rows),
            "stage6000_residual": _merge_stage6000(trusted_rows, labelled=True),
            "thresholds": {channel: choose_threshold(
                tune_all, holdout_all, channel, min_correct=min_correct)
                for channel in THRESHOLD_CHANNELS},
        },
        "objects": panels,
    }


def load_producer_provenance(input_path, pseudo_gt_path):
    input_path = Path(input_path)
    suffix = ".pem-score-analysis.jsonl"
    if not input_path.name.endswith(suffix):
        raise AnalysisInputError("analysis input filename must end with .pem-score-analysis.jsonl")
    path = input_path.with_name(input_path.name[:-len(suffix)] + ".provenance.json")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise AnalysisInputError(f"missing or invalid producer provenance: {path}") from exc
    output = ((value.get("outputs") or {}).get("pem_score_analysis") or {})
    references = value.get("references") or {}
    correctness = ((value.get("run") or {}).get("correctness") or {})
    if output.get("sha256") != sha256(input_path):
        raise AnalysisInputError("producer provenance score artifact hash mismatch")
    if Path(output.get("file", "")).resolve() != input_path.resolve():
        raise AnalysisInputError("producer provenance score artifact path mismatch")
    if references.get("pseudo_gt_sha256") != sha256(pseudo_gt_path):
        raise AnalysisInputError("producer provenance pseudo-GT hash mismatch")
    trajectory = Path(references.get("trajectory", ""))
    if (not trajectory.is_file()
            or references.get("trajectory_sha256") != sha256(trajectory)):
        raise AnalysisInputError("producer provenance trajectory hash mismatch")
    if not isinstance(value.get("dataset_id"), str) or not value["dataset_id"]:
        raise AnalysisInputError("producer provenance dataset_id is missing")
    if (not _finite(correctness.get("rotation_threshold_deg"))
            or not _finite(correctness.get("translation_threshold_mm"))
            or not isinstance(correctness.get("symmetry_step_deg"), int)
            or not isinstance(correctness.get("symmetry_axes"), dict)):
        raise AnalysisInputError("producer provenance correctness policy is invalid")
    if output.get("selection_method") != "geometry_only" or output.get("thresholds_active") is not False:
        raise AnalysisInputError("producer provenance violates analysis-only selection policy")
    return path, value


def load_selection_parity(path, producer_provenance):
    parity_path = Path(path)
    try:
        value = json.loads(parity_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise AnalysisInputError(f"missing or invalid selection parity: {parity_path}") from exc
    detections = producer_provenance[1]["outputs"]["detections"]
    baseline = value.get("baseline") or {}
    instrumented = value.get("instrumented") or {}
    baseline_path = Path(baseline.get("path", ""))
    instrumented_path = Path(instrumented.get("path", ""))
    if (value.get("status") != "pass" or value.get("mismatches") != []
            or value.get("rows_compared") != detections.get("rows")
            or instrumented.get("sha256") != detections.get("sha256")
            or instrumented_path.resolve() != Path(detections.get("file", "")).resolve()
            or not baseline_path.is_file() or baseline.get("sha256") != sha256(baseline_path)
            or not instrumented_path.is_file()
            or instrumented.get("sha256") != sha256(instrumented_path)):
        raise AnalysisInputError("selection parity does not authenticate producer detections")
    return parity_path, value


def write_report(result, out_dir, input_path, pseudo_gt_path, producer_provenance=None,
                 selection_parity=None):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    result = {**result, "provenance": {
        "compact_analysis_input": str(Path(input_path).resolve()),
        "compact_analysis_sha256": sha256(input_path),
        "pseudo_gt": str(Path(pseudo_gt_path).resolve()),
        "pseudo_gt_sha256": sha256(pseudo_gt_path),
        **({"producer_provenance": str(Path(producer_provenance[0]).resolve()),
            "producer_provenance_sha256": sha256(producer_provenance[0]),
            "trajectory_sha256": producer_provenance[1]["references"]["trajectory_sha256"]}
           if producer_provenance else {}),
        **({"selection_parity": str(selection_parity[0].resolve()),
            "selection_parity_sha256": sha256(selection_parity[0]),
            "selection_parity_status": selection_parity[1]["status"]}
           if selection_parity else {}),
    }}
    contents = {
        "analysis.json": json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        "analysis-data.js": "window.PEM_SCORE_ANALYSIS=" + json.dumps(
            result, ensure_ascii=False, separators=(",", ":")) + ";\n",
        "app.css": (TEMPLATE_DIR / "app.css").read_text(encoding="utf-8"),
        "app.js": (TEMPLATE_DIR / "app.js").read_text(encoding="utf-8"),
    }
    explorer = Path(__file__).resolve().parents[1] / "output" / "pem_explorer" / "index.html"
    explorer_href = os.path.relpath(explorer, out).replace(os.sep, "/")
    contents["index.html"] = (TEMPLATE_DIR / "index.html").read_text(
        encoding="utf-8").replace("__EXPLORER_HREF__", explorer_href)
    for name, content in contents.items():
        temporary = out / f".{name}.tmp"
        temporary.write_text(content, encoding="utf-8")
        os.replace(temporary, out / name)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True,
                        help="*.pem-score-analysis.jsonl from temp/verify_eval.py")
    parser.add_argument("--pseudo-gt", required=True)
    parser.add_argument("--dataset-id", default="",
                        help="optional assertion; authenticated producer dataset_id is authoritative")
    parser.add_argument("--min-correct", type=int, default=20)
    parser.add_argument("--out", default="output/pem_score_distributions")
    parser.add_argument("--selection-parity",
                        help="optional passed selection-parity.json to bind into provenance")
    args = parser.parse_args()
    pseudo_gt = json.loads(Path(args.pseudo_gt).read_text(encoding="utf-8"))
    producer_provenance = load_producer_provenance(args.input, args.pseudo_gt)
    selection_parity = (load_selection_parity(args.selection_parity, producer_provenance)
                        if args.selection_parity else None)
    producer_dataset_id = producer_provenance[1]["dataset_id"]
    if args.dataset_id and args.dataset_id != producer_dataset_id:
        raise AnalysisInputError("requested dataset_id disagrees with producer provenance")
    rows = load_jsonl(args.input)
    if producer_provenance[1]["outputs"]["pem_score_analysis"].get("rows") != len(rows):
        raise AnalysisInputError("producer provenance score row count mismatch")
    result = analyze(rows, pseudo_gt, producer_dataset_id, args.min_correct)
    policy = producer_provenance[1]["run"]["correctness"]
    definitions = result["definitions"]
    expected_symmetry = {
        name: policy["symmetry_axes"].get(name)
        for name in definitions["symmetry_policy"]
    }
    observed_symmetry = {
        name: value["axis"] for name, value in definitions["symmetry_policy"].items()
    }
    observed_objects = {row["object"] for row in rows}
    if (definitions["correctness_thresholds"] != {
            "rotation_threshold_deg": policy["rotation_threshold_deg"],
            "translation_threshold_mm": policy["translation_threshold_mm"]}
            or any(name in observed_objects and axis != expected_symmetry[name]
                   for name, axis in observed_symmetry.items())
            or any(value["step_deg"] != policy["symmetry_step_deg"]
                   for value in definitions["symmetry_policy"].values())):
        raise AnalysisInputError("row correctness policy disagrees with producer provenance")
    definitions["symmetry_policy"] = {
        name: {"axis": axis, "step_deg": policy["symmetry_step_deg"]}
        for name, axis in expected_symmetry.items()
    }
    # Detect replacement of either authenticated input during analysis.
    load_producer_provenance(args.input, args.pseudo_gt)
    write_report(result, args.out, args.input, args.pseudo_gt, producer_provenance,
                 selection_parity)
    print(json.dumps({"out": str(Path(args.out).resolve()),
                      "objects": len(result["objects"]),
                      "trusted_evaluation_detections":
                          result["overall"]["trusted_evaluation_detection_count"],
                      "thresholds_active": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
