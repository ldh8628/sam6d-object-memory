#!/usr/bin/env python3
"""Stream a large PEM diagnostic into the compact pose-reference JSONL schema."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path


FIELDS = ("stamp_ns", "i", "object", "score", "R", "t_mm", "bbox")
REQUIRED = ("stamp_ns", "object", "R", "t_mm", "bbox")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def extract(source, output, replace=False, dataset_id="", dataset_manifest=""):
    source, output = Path(source), Path(output)
    if not source.is_file() or source.stat().st_size <= 0:
        raise ValueError(f"source is missing or empty: {source}")
    provenance_path = output.with_suffix(output.suffix + ".provenance.json")
    if output.exists() and not replace:
        raise ValueError(f"refusing to overwrite output: {output}")
    if provenance_path.exists() and not replace:
        raise ValueError(f"refusing to overwrite provenance: {provenance_path}")
    if bool(dataset_id) != bool(dataset_manifest):
        raise ValueError("dataset_id and dataset_manifest must be supplied together")
    manifest = None
    source_digest = database_digest = metadata_digest = None
    if dataset_manifest:
        manifest_path = Path(dataset_manifest).resolve()
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid dataset manifest: {exc}") from exc
        if not isinstance(manifest, dict) or manifest.get("dataset_id") != dataset_id:
            raise ValueError("dataset manifest dataset_id mismatch")
        bag_info = manifest.get("bag")
        if not isinstance(bag_info, dict) or not isinstance(bag_info.get("database"), str):
            raise ValueError("dataset manifest bag.database is required")
        database = manifest_path.parent / bag_info["database"]
        metadata = manifest_path.parent / "metadata.yaml"
        if not database.is_file() or not metadata.is_file():
            raise ValueError("dataset database or metadata is missing")
        source_digest = sha256(source)
        database_digest = sha256(database)
        metadata_digest = sha256(metadata)
        diagnostic_provenance_path = source.with_name(
            source.stem + ".provenance.json")
        try:
            diagnostic_provenance = json.loads(
                diagnostic_provenance_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid diagnostic provenance: {exc}") from exc
        snapshot = diagnostic_provenance.get("dataset_snapshot")
        outputs = diagnostic_provenance.get("outputs")
        detections = outputs.get("detections") if isinstance(outputs, dict) else None
        if (diagnostic_provenance.get("kind") != "SAM-6D diagnostic run"
                or diagnostic_provenance.get("dataset_id") != dataset_id
                or diagnostic_provenance.get("optical_frame") !=
                (manifest.get("camera") or {}).get("optical_frame")
                or not isinstance(snapshot, dict) or not isinstance(detections, dict)
                or snapshot.get("bag_database") != bag_info["database"]
                or snapshot.get("bag_database_sha256") != database_digest
                or snapshot.get("metadata_sha256") != metadata_digest
                or Path(str(detections.get("file", ""))).resolve() != source.resolve()
                or detections.get("sha256") != source_digest):
            raise ValueError("diagnostic provenance does not attest this dataset/source")
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{output.name}.", dir=output.parent)
    count = 0
    try:
        with source.open(encoding="utf-8") as inp, os.fdopen(
                fd, "w", encoding="utf-8") as out:
            for number, line in enumerate(inp, 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{source}:{number}: invalid JSON: {exc}") from exc
                missing = [key for key in REQUIRED if key not in row]
                if missing:
                    raise ValueError(
                        f"{source}:{number}: missing fields: {', '.join(missing)}")
                out.write(json.dumps({key: row[key] for key in FIELDS if key in row},
                                     ensure_ascii=False, separators=(",", ":")) + "\n")
                count += 1
        os.replace(temporary, output)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    result = {"rows": count, "bytes": output.stat().st_size, "sha256": sha256(output)}
    if manifest is not None:
        provenance = {
            "schema_version": 1,
            "kind": "compact PEM pose reference",
            "dataset_id": dataset_id,
            "dataset_manifest": {
                "file": str(Path(dataset_manifest).resolve()),
                "dataset_id": dataset_id,
                "note": "mutable manifest path; dataset_snapshot hashes are the trust anchor",
            },
            "dataset_snapshot": {
                "bag_database": bag_info["database"],
                "bag_database_sha256": database_digest,
                "metadata_sha256": metadata_digest,
            },
            "source": {"file": str(source.resolve()), "sha256": source_digest},
            "diagnostic_provenance": {
                "file": str(diagnostic_provenance_path.resolve()),
                "sha256": sha256(diagnostic_provenance_path),
                "content": diagnostic_provenance,
            },
            "output": {"file": str(output.resolve()), "sha256": result["sha256"],
                       "rows": count, "fields": list(FIELDS)},
        }
        provenance_path.write_text(
            json.dumps(provenance, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        result["provenance"] = str(provenance_path)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("source")
    ap.add_argument("output")
    ap.add_argument("--replace", action="store_true")
    ap.add_argument("--dataset-id", default="")
    ap.add_argument("--dataset-manifest", default="")
    args = ap.parse_args()
    try:
        result = extract(args.source, args.output, args.replace,
                         args.dataset_id, args.dataset_manifest)
    except (OSError, ValueError) as exc:
        ap.error(str(exc))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
