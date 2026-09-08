import importlib.util
import json
from pathlib import Path

import pytest


PATH = Path(__file__).resolve().parents[1] / "tools" / "extract_pose_reference.py"
SPEC = importlib.util.spec_from_file_location("extract_pose_reference", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def test_extract_keeps_only_reference_fields_atomically(tmp_path):
    source = tmp_path / "diagnostic.jsonl"
    source.write_text(json.dumps({
        "stamp_ns": 1, "i": 2, "object": "milk", "score": 0.9,
        "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t_mm": [0, 0, 1],
        "bbox": [0, 0, 10, 10], "verify": {"huge": [1, 2, 3]},
    }) + "\n", encoding="utf-8")
    output = tmp_path / "reference.jsonl"

    result = MOD.extract(source, output)

    row = json.loads(output.read_text(encoding="utf-8"))
    assert "verify" not in row and set(row) == set(MOD.FIELDS)
    assert result["rows"] == 1 and len(result["sha256"]) == 64
    with pytest.raises(ValueError, match="overwrite"):
        MOD.extract(source, output)


def test_extract_rejects_missing_pose_field_without_publishing(tmp_path):
    source = tmp_path / "bad.jsonl"
    source.write_text('{"stamp_ns":1,"object":"milk"}\n', encoding="utf-8")
    output = tmp_path / "reference.jsonl"

    with pytest.raises(ValueError, match="missing fields"):
        MOD.extract(source, output)
    assert not output.exists()


def test_extract_attests_dataset_source_and_output(tmp_path):
    source = tmp_path / "diagnostic.jsonl"
    source.write_text(json.dumps({
        "stamp_ns": 1, "object": "milk",
        "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t_mm": [0, 0, 1],
        "bbox": [0, 0, 10, 10],
    }) + "\n", encoding="utf-8")
    database = tmp_path / "sample.db3"
    database.write_bytes(b"database")
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text("metadata", encoding="utf-8")
    manifest = tmp_path / "dataset_manifest.json"
    manifest.write_text(json.dumps({
        "dataset_id": "sample", "camera": {"optical_frame": "camera"},
        "bag": {"database": database.name},
    }), encoding="utf-8")
    diagnostic_provenance = {
        "kind": "SAM-6D diagnostic run", "dataset_id": "sample",
        "optical_frame": "camera",
        "dataset_snapshot": {
            "bag_database": database.name,
            "bag_database_sha256": MOD.sha256(database),
            "metadata_sha256": MOD.sha256(metadata),
        },
        "outputs": {"detections": {
            "file": str(source.resolve()), "sha256": MOD.sha256(source), "rows": 1,
        }},
    }
    (tmp_path / "diagnostic.provenance.json").write_text(
        json.dumps(diagnostic_provenance), encoding="utf-8")
    output = tmp_path / "reference.jsonl"

    result = MOD.extract(source, output, dataset_id="sample",
                         dataset_manifest=manifest)
    provenance = json.loads(Path(result["provenance"]).read_text(encoding="utf-8"))

    assert provenance["dataset_id"] == "sample"
    assert provenance["source"]["sha256"] == MOD.sha256(source)
    assert provenance["output"]["sha256"] == MOD.sha256(output)
    assert provenance["diagnostic_provenance"]["content"]["dataset_id"] == "sample"
