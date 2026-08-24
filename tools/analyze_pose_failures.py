#!/usr/bin/env python3
"""Classify longcircle2 failures and persist evidence-graded findings."""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path


def jsonl(path):
    with Path(path).open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def cause(row):
    trans = float(row["translation_error_m"])
    rot = float(row["rotation_error_deg"])
    if trans > 0.50:
        return "ism_false_positive_or_wrong_instance"
    if trans > 0.10:
        return "pem_translation_or_mask_outlier"
    if rot > 45.0:
        return "pem_orientation_flip"
    return "pem_rotation_outside_30deg"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--failures", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--method", default="geometry_texture")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    failures = [r for r in jsonl(args.failures) if r["method"] == args.method]
    by_cause, by_object, verdict = Counter(), Counter(), Counter()
    enriched = []
    for row in failures:
        label = cause(row)
        by_cause[label] += 1
        by_object[(row["object"], label)] += 1
        verdict[(row.get("verify") or {}).get("verdict", "NO_VERDICT")] += 1
        enriched.append({**row, "root_cause": label})

    rejected = []
    rejected_counts = Counter()
    error_frames = 0
    for frame in jsonl(args.frames):
        diagnostics = frame.get("diagnostics") or {}
        if diagnostics.get("pem_error"):
            error_frames += 1
        for item in diagnostics.get("pem_candidates", []):
            reason = item.get("input_rejection")
            if not reason:
                continue
            rejected_counts[(item["object"], reason)] += 1
            rejected.append({
                "method": "pem_input",
                "stamp_ns": int(frame["stamp_ns"]),
                "frame_index": int(frame["i"]),
                "object": item["object"],
                "category": "pem_input_rejected",
                "root_cause": reason,
                **item,
            })

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "pose_failures_classified.jsonl").write_text(
        "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in enriched), encoding="utf-8")
    (out / "pem_input_rejections.jsonl").write_text(
        "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rejected), encoding="utf-8")

    summary = {
        "method": args.method,
        "pose_failure_count": len(failures),
        "pose_failures_by_cause": dict(by_cause),
        "pose_failures_by_verify_verdict": dict(verdict),
        "pem_error_frames": error_frames,
        "pem_rejected_candidates": len(rejected),
        "pem_rejections_by_object_reason": {
            f"{obj}:{reason}": count for (obj, reason), count in sorted(rejected_counts.items())
        },
    }
    (out / "failure_analysis.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    lines = ["# longcircle2 failure analysis", "",
             "## Confirmed", "",
             f"- {len(failures)} evaluated poses failed the ≤30°/≤10 cm acceptance rule.",
             f"- {error_frames} frames had every ISM candidate rejected before PEM; "
             f"{len(rejected)} individual candidates were rejected in total.",
             "- PEM input rejection is triggered by either ≤32 valid-depth mask pixels or fewer than "
             "4 points inside the CAD-radius gate; these exact counters are preserved per frame.", "",
             "## Deduced classification", "",
             "| category | count | interpretation |",
             "|---|---:|---|"]
    descriptions = {
        "ism_false_positive_or_wrong_instance": "map translation error >50 cm; usually a wrong visual instance/ISM false positive",
        "pem_translation_or_mask_outlier": "translation error 10–50 cm; mask/depth or PEM translation outlier",
        "pem_orientation_flip": "translation is consistent but rotation error >45°; PEM pose-mode flip",
        "pem_rotation_outside_30deg": "translation is consistent but rotation misses the 30° target",
    }
    for label, count in by_cause.most_common():
        lines.append(f"| {label} | {count} | {descriptions[label]} |")
    lines += ["", "## Verification verdict on failed poses", "",
              "| verdict | failed poses |", "|---|---:|"]
    for label, count in verdict.most_common():
        lines.append(f"| {label} | {count} |")
    lines += ["", "## PEM input rejection", "",
              "| object / reason | count |", "|---|---:|"]
    for (obj, reason), count in sorted(rejected_counts.items(), key=lambda x: -x[1]):
        lines.append(f"| {obj} / {reason} | {count} |")
    lines += ["", "## Hypotheses requiring visual confirmation", "",
              "- Translation errors above 50 cm are classified as ISM false positives or wrong-instance matches; "
              "the saved contact sheets provide the confirmation set.",
              "- `radius_inliers<4` concentrated on saffron can be a false-positive symptom or a CAD-scale/mask-depth "
              "mismatch. The rejected-frame set must distinguish these two cases."]
    (out / "FAILURE_ANALYSIS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
