import json

from tools.compare_anchor_modes import compare


def _write(path, name, rows):
    (path / name).write_text("\n".join(json.dumps(row) for row in rows) + "\n",
                             encoding="utf-8")


def test_compare_reports_coverage_errors_stability_and_missing_ism_recovery(tmp_path):
    associated, always = tmp_path / "associated", tmp_path / "always"
    associated.mkdir(); always.mkdir()
    for path in (associated, always):
        _write(path, "frames.jsonl", [{"frame_seq": i} for i in range(3)])
    base = {"object": "milk", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "t_mm": [0, 0, 1000], "correct": True}
    _write(associated, "detections.jsonl", [{**base, "frame_seq": 0}])
    _write(always, "detections.jsonl", [
        {**base, "frame_seq": 0},
        {**base, "frame_seq": 1, "pose_source": "slam_anchor", "correct": False,
         "anchor_diagnostic": {"ism_present": False}},
    ])
    result = compare(associated, always)
    assert result["ism_associated"]["output_coverage"] == 1 / 3
    assert result["fov_always"]["output_coverage"] == 2 / 3
    assert result["fov_always"]["incorrect_output_count"] == 1
    assert result["fov_always"]["output_accuracy"] == 0.5
    assert result["fov_always"]["ism_missing_recovery_count"] == 1
    assert result["fov_always"]["pose_stability"]["rotation_step_median_deg"] == 0.0
    assert result["fov_always"]["by_object"]["milk"]["output_count"] == 2
