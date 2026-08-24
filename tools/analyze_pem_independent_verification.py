#!/usr/bin/env python3
"""Summarize geometry-only selection and independent verification channels."""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def load_jsonl(path):
    with Path(path).open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)


def percentiles(values):
    if not values:
        return None
    a = np.asarray(values, float)
    return {f"p{p}": round(float(np.percentile(a, p)), 5)
            for p in (0, 10, 25, 50, 75, 90, 100)}


def summarize(rows):
    groups = defaultdict(lambda: {
        "count": 0, "selection": Counter(), "unavailable": 0,
        "texture_ranks": [], "sizes": [], "ious": [], "coverages": [],
    })
    wanted = {(1785831526041608960, "choco_hazelnut_high"),
              (1785831518074359552, "saffron"),
              (1785831536184341504, "saffron")}
    cases = {}
    for row in rows:
        verify = row.get("verify") or {}
        texture = verify.get("texture") or {}
        shape = verify.get("shape") or {}
        key = (int(row.get("stamp_ns", -1)), row.get("object"))
        if key in wanted:
            cases[key] = {"stamp_ns": key[0], "object": key[1],
                          "texture": texture, "shape": shape}
        for name in ("all", row["object"]):
            state = groups[name]
            state["count"] += 1
            state["selection"][str(verify.get("selection_method"))] += 1
            if texture.get("rank") is not None:
                state["texture_ranks"].append(int(texture["rank"]))
            if shape.get("size_ratio") is None:
                state["unavailable"] += 1
            else:
                state["sizes"].append(float(shape["size_ratio"]))
                state["ious"].append(float(shape.get("mask_iou", 0.0)))
                state["coverages"].append(float(shape.get("coverage", 0.0)))
    out = {}
    for name, state in sorted(groups.items()):
        texture_ranks = state["texture_ranks"]
        sizes, ious, coverages = state["sizes"], state["ious"], state["coverages"]
        n = state["count"]
        out[name] = {
            "count": n,
            "selection_method_counts": dict(sorted(state["selection"].items())),
            "verification_unavailable": state["unavailable"],
            "texture_geo_winner_rank": {
                "distribution": percentiles(texture_ranks),
                "top1": sum(rank == 0 for rank in texture_ranks),
                "top5": sum(rank < 5 for rank in texture_ranks),
                "top10": sum(rank < 10 for rank in texture_ranks),
            },
            "size_ratio": {
                "distribution": percentiles(sizes),
                "fail_gt_1.00": sum(value > 1.00 for value in sizes),
                "fail_gt_1.05": sum(value > 1.05 for value in sizes),
                "fail_gt_1.10": sum(value > 1.10 for value in sizes),
            },
            "mask_iou": {
                "distribution": percentiles(ious),
                "below_0.30": sum(value < .30 for value in ious),
                "below_0.40": sum(value < .40 for value in ious),
                "below_0.50": sum(value < .50 for value in ious),
            },
            "coverage": {
                "distribution": percentiles(coverages),
                "below_0.30": sum(value < .30 for value in coverages),
                "below_0.50": sum(value < .50 for value in coverages),
            },
        }
    case_rows = [cases.get(key, {"stamp_ns": key[0], "object": key[1],
                                 "status": "not_detected"}) for key in sorted(wanted)]
    return out, case_rows


def legacy_summary(rows):
    used = [int((row.get("verify") or {}).get("geo_rank_used", 0)) for row in rows]
    return {"count": len(used), "geometry_top1_kept": sum(rank == 0 for rank in used),
            "combined_reranked": sum(rank != 0 for rank in used),
            "geo_rank_used_distribution": percentiles(used)}


def markdown(report):
    overall = report["independent"]["all"]
    lines = ["# PEM geometry-only + independent verification", "",
             f"- detections: {overall['count']}",
             f"- strict size failures (>1.00): {overall['size_ratio']['fail_gt_1.00']}",
             f"- tolerance failures (>1.05 / >1.10): "
             f"{overall['size_ratio']['fail_gt_1.05']} / {overall['size_ratio']['fail_gt_1.10']}",
             f"- mask IoU below .30/.40/.50: {overall['mask_iou']['below_0.30']} / "
             f"{overall['mask_iou']['below_0.40']} / {overall['mask_iou']['below_0.50']}",
             f"- geometry winner texture top-1/top-5/top-10: "
             f"{overall['texture_geo_winner_rank']['top1']} / "
             f"{overall['texture_geo_winner_rank']['top5']} / "
             f"{overall['texture_geo_winner_rank']['top10']}", ""]
    if report.get("legacy"):
        lines += [f"- legacy combined reranked away from geometry top-1: "
                  f"{report['legacy']['combined_reranked']} / {report['legacy']['count']}", ""]
    lines += ["No validated SAM-camera GT is available; these are filter distributions, "
              "not accuracy measurements.", "", "## Representative cases", "",
              "```json", json.dumps(report["representative_cases"], ensure_ascii=False,
                                      indent=2), "```", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--detections", required=True)
    parser.add_argument("--legacy", default="")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    independent, cases = summarize(load_jsonl(args.detections))
    report = {
        "source": args.detections,
        "independent": independent,
        "legacy": legacy_summary(load_jsonl(args.legacy)) if args.legacy else None,
        "representative_cases": cases,
        "ground_truth_notice": "extrinsic_missing; distributions are not accuracy",
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "verification-summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (out / "README.md").write_text(markdown(report), encoding="utf-8")
    print(json.dumps({"out": str(out), "detections": independent["all"]["count"]},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
