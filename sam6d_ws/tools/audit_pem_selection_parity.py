#!/usr/bin/env python3
"""Audit that PEM analysis instrumentation does not change selected poses."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from itertools import zip_longest
from pathlib import Path
from typing import Any, Iterator


def _rows(path: Path) -> Iterator[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(f"{path}:{line_number}: invalid JSON") from error


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _flatten(matrix: Any) -> list[float]:
    if not isinstance(matrix, list):
        return []
    return [float(value) for row in matrix for value in (row if isinstance(row, list) else [row])]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("instrumented", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--rotation-tolerance", type=float, default=0.0)
    parser.add_argument("--translation-tolerance-mm", type=float, default=0.0)
    args = parser.parse_args()
    if (not math.isfinite(args.rotation_tolerance) or args.rotation_tolerance < 0
            or not math.isfinite(args.translation_tolerance_mm)
            or args.translation_tolerance_mm < 0):
        parser.error("tolerances must be finite and nonnegative")

    mismatches: list[dict[str, Any]] = []
    max_rotation_difference = 0.0
    max_translation_difference = 0.0
    row_count = 0
    for row_count, pair in enumerate(
        zip_longest(_rows(args.baseline), _rows(args.instrumented)), 1
    ):
        baseline, instrumented = pair
        if baseline is None or instrumented is None:
            mismatches.append({"row": row_count, "reason": "row_count_mismatch"})
            break
        identity = [baseline.get("i"), baseline.get("stamp_ns"), baseline.get("object")]
        other_identity = [instrumented.get("i"), instrumented.get("stamp_ns"), instrumented.get("object")]
        if identity != other_identity:
            mismatches.append({
                "row": row_count, "reason": "identity_mismatch",
                "baseline": identity, "instrumented": other_identity,
            })
            continue
        baseline_index = baseline.get("verify", {}).get("selected_proposal6000_index")
        instrumented_index = instrumented.get("verify", {}).get("selected_proposal6000_index")
        baseline_rotation, instrumented_rotation = (
            _flatten(baseline.get("R")), _flatten(instrumented.get("R")))
        baseline_translation, instrumented_translation = (
            _flatten(baseline.get("t_mm")), _flatten(instrumented.get("t_mm")))
        pose_values = (baseline_rotation + instrumented_rotation
                       + baseline_translation + instrumented_translation)
        if (len(baseline_rotation) != 9 or len(instrumented_rotation) != 9
                or len(baseline_translation) != 3 or len(instrumented_translation) != 3
                or any(not math.isfinite(value) for value in pose_values)):
            mismatches.append({
                "row": row_count, "identity": identity,
                "reason": "nonfinite_or_malformed_pose",
            })
            continue
        rotation_differences = [abs(left - right) for left, right in zip(
            baseline_rotation, instrumented_rotation, strict=True)]
        translation_differences = [abs(left - right) for left, right in zip(
            baseline_translation, instrumented_translation, strict=True)]
        rotation_difference = max(rotation_differences, default=math.inf)
        translation_difference = max(translation_differences, default=math.inf)
        max_rotation_difference = max(max_rotation_difference, rotation_difference)
        max_translation_difference = max(max_translation_difference, translation_difference)
        if (baseline_index != instrumented_index
                or rotation_difference > args.rotation_tolerance
                or translation_difference > args.translation_tolerance_mm):
            mismatches.append({
                "row": row_count, "identity": identity,
                "baseline_candidate_index": baseline_index,
                "instrumented_candidate_index": instrumented_index,
                "max_rotation_element_difference": rotation_difference,
                "max_translation_difference_mm": translation_difference,
            })
        if len(mismatches) >= 20:
            break

    if row_count == 0:
        mismatches.append({"row": 0, "reason": "empty_inputs"})
    report = {
        "schema_version": 1,
        "status": "pass" if not mismatches else "fail",
        "definition": "Ordered identity, geometry-selected 6000 index, R, and t_mm parity",
        "baseline": {"path": str(args.baseline.resolve()), "sha256": _sha256(args.baseline)},
        "instrumented": {"path": str(args.instrumented.resolve()), "sha256": _sha256(args.instrumented)},
        "rows_compared": row_count,
        "rotation_tolerance": args.rotation_tolerance,
        "translation_tolerance_mm": args.translation_tolerance_mm,
        "max_rotation_element_difference": max_rotation_difference,
        "max_translation_difference_mm": max_translation_difference,
        "mismatches": mismatches,
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.out.with_name(f".{args.out.name}.tmp-{os.getpid()}")
        temporary.write_text(rendered, encoding="utf-8")
        temporary.replace(args.out)
    print(rendered, end="")
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
