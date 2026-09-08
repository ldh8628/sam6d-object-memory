import importlib.util
import json
from pathlib import Path


PATH = Path(__file__).resolve().parents[1] / "tools" / "analyze_pem_candidate_score_distributions.py"
SPEC = importlib.util.spec_from_file_location("analyze_pem_scores", PATH)
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


TRUSTED = ["Bear", "Dinosaur", "Mugcup_high", "Sauce_high",
           "Sikhye_high", "milk", "saffron"]
UNTRUSTED = ["Febreze_high", "choco_hazelnut_high"]


def candidate(index, correct, geo, texture, iou):
    return {
        "index300": index, "proposal6000_index": index,
        "geometry_score": geo,
        "geometry_score_detection_normalized": geo,
        "texture_score": texture, "mask_iou": iou,
        "projection_valid": iou is not None,
        "missing_reason": None if iou is not None else "invalid_projection",
        "geometry_selected": index == 0,
        "gt_status": "available", "correct": correct,
        "rotation_error_deg": 1.0 if correct else 90.0,
        "translation_error_mm": 1.0,
    }


def row(name, stamp, role="evaluation_frame", missing=False):
    correctness = {"rotation_threshold_deg": 30.0,
                   "translation_threshold_mm": 100.0,
                   "symmetry_axis": None, "symmetry_step_deg": 10}
    def compact(count):
        return {"count": count, "missing_count": 0,
                "edges": [0.0, 1.0], "bins": [count], "overflow": 0,
                "quantiles": {"min": .5, "max": .5} if count else {}}
    candidates = [candidate(0, True, .9, .8, None if missing else .7),
                  candidate(1, False, .1, .2, .1)]
    return {
        "schema_version": 1, "stamp_ns": stamp, "frame_index": stamp,
        "object": name, "reference_role": role,
        "stage6000": {
            "score_name": "initial_3point_residual", "candidate_count": 6000,
            "units": "normalized_object_radius", "correctness": correctness,
            "gt_status": "available", "overall": compact(6000),
            "correct": compact(1), "incorrect": compact(5999),
            "invalid_pose": compact(0),
        },
        "stage300": {"schema_version": 1, "selection_method": "geometry_only",
                     "candidate_count": 2, "selected_index300": 0,
                     "correctness": correctness,
                     "channels": {}, "candidates": candidates},
    }


def fixture():
    pseudo = {name: {"trusted": True,
                     "reference_frames": ([{"stamp_ns": 5}] if name == "Bear" else [])}
              for name in TRUSTED}
    pseudo.update({name: {"trusted": False, "unavailable_reason": "cluster_share_too_low",
                          "reference_frames": []} for name in UNTRUSTED})
    rows = []
    for name in TRUSTED + UNTRUSTED:
        rows.extend([row(name, 10), row(name, 20)])
    rows.append(row("Bear", 5, "reference_member"))
    return rows, pseudo


def test_analysis_excludes_reference_members_and_never_thresholds_untrusted():
    rows, pseudo = fixture()
    result = MOD.analyze(rows, pseudo, min_correct=1)
    assert result["definitions"]["trusted_objects"] == sorted(TRUSTED)
    assert result["definitions"]["untrusted_objects"] == sorted(UNTRUSTED)
    assert result["objects"]["Bear"]["excluded_reference_member_count"] == 1
    assert result["objects"]["Bear"]["input_detection_count"] == 2
    assert result["objects"]["Bear"]["thresholds"]["geometry_score"]["status"] == "proposed"
    for name in UNTRUSTED:
        panel = result["objects"][name]
        assert panel["status"] == "gt_trust_insufficient"
        assert panel["thresholds"] == {}
        assert "unlabelled" in panel["channels"]["geometry_score"]


def test_blocked_split_and_threshold_direction_do_not_leak_holdout():
    rows = [row("Bear", stamp) for stamp in (30, 10, 40, 20)]
    tune, holdout = MOD.blocked_split(rows)
    assert [x["stamp_ns"] for x in tune] == [10, 20]
    assert [x["stamp_ns"] for x in holdout] == [30, 40]
    proposed = MOD.choose_threshold(tune, holdout, "geometry_score", min_correct=1)
    assert proposed["status"] == "proposed"
    assert proposed["direction"] == "keep_score_greater_than_or_equal"
    assert proposed["tune"]["correct_candidate_retention"] == 1.0
    assert proposed["holdout"]["detection_level_recall"] == 1.0


def test_missing_scores_are_separate_not_zero_and_can_make_holdout_fail():
    rows = [row("Bear", 10, missing=True), row("Bear", 20, missing=True)]
    panel = MOD.analyze(rows, {"Bear": {"trusted": True}}, min_correct=1)["objects"]["Bear"]
    iou = panel["channels"]["mask_iou"]
    assert iou["correct"]["count"] == 0
    assert iou["correct"]["missing_count"] == 2
    assert panel["thresholds"]["mask_iou"]["status"] == "insufficient_data"


def test_missing_correct_scores_consume_retention_budget():
    tune = [row("Bear", stamp) for stamp in range(20)]
    holdout = [row("Bear", 100)]
    tune[0]["stage300"]["candidates"][0]["mask_iou"] = None
    proposed = MOD.choose_threshold(
        tune, holdout, "mask_iou", minimum_retention=.99, min_correct=1)
    assert proposed["status"] == "insufficient_data"
    assert proposed["reason"] == "missing_correct_scores_exceed_retention_budget"


def test_gt_unavailable_candidates_are_not_mislabeled_as_incorrect():
    tune = row("Bear", 10)
    holdout = row("Bear", 20)
    for candidate_value in tune["stage300"]["candidates"]:
        candidate_value.update({"gt_status": "unavailable", "correct": None})
    tune["stage6000"].update({"gt_status": "unavailable", "correct": None,
                              "incorrect": None})
    panel = MOD.analyze([tune, holdout], {"Bear": {"trusted": True}},
                        min_correct=1)["objects"]["Bear"]
    geometry = panel["channels"]["geometry_score"]
    assert geometry["gt_unavailable_count"] == 2
    assert geometry["incorrect"]["count"] == 1
    metric = MOD.threshold_metrics([tune, holdout], "geometry_score", .5)
    assert metric["incorrect_candidate_count"] == 1
    assert metric["gt_unavailable_candidate_count"] == 2
    assert panel["stage_attrition"]["gt_unavailable_detections"] == 1


def test_invalid_pose_candidates_are_accounted_separately():
    value = row("Bear", 10)
    invalid = value["stage300"]["candidates"][1]
    invalid.update({"candidate_valid": False, "correct": None,
                    "geometry_score": None, "texture_score": None,
                    "mask_iou": None})
    channel = MOD._channel_distributions([value], "geometry_score", labelled=True)
    metric = MOD.threshold_metrics([value], "geometry_score", .5)
    assert channel["invalid_pose_count"] == 1
    assert channel["gt_unavailable_count"] == 0
    assert metric["invalid_pose_candidate_count"] == 1
    assert metric["gt_unavailable_candidate_count"] == 0


def test_overall_survivor_count_keeps_same_frame_objects_separate():
    bear = row("Bear", 10)
    dinosaur = row("Dinosaur", 10)
    metrics = MOD.threshold_metrics(
        [bear, dinosaur], "geometry_score", .5)
    assert metrics["eligible_detection_count"] == 2
    assert metrics["surviving_candidates_mean"] == 1.0


def test_stage_attrition_counts_6000_to_300_loss():
    lost = row("Bear", 10)
    for item in lost["stage300"]["candidates"]:
        item["correct"] = False
    kept = row("Bear", 20)
    value = MOD._stage_attrition([lost, kept])
    assert value["stage6000_with_correct"] == 2
    assert value["stage300_with_correct"] == 1
    assert value["lost_6000_to_300"] == 1


def test_stage6000_merge_uses_compact_counts_without_candidate_materialization():
    value = row("Bear", 10)
    aggregate = {
        "count": 10_000_003, "missing_count": 7,
        "edges": [0.0, 0.1, 1.0], "bins": [10_000_000, 2],
        "overflow": 1, "quantiles": {},
    }
    value["stage6000"] = {"correct": aggregate, "incorrect": aggregate,
                          "overall": aggregate}
    merged = MOD._merge_stage6000([value], labelled=True)
    assert merged["correct"]["count"] == 10_000_003
    assert sum(x["count"] for x in merged["correct"]["histogram"]) + \
        merged["correct"]["overflow"] == merged["correct"]["count"]
    assert merged["correct"]["missing_count"] == 7
    assert len(merged["correct"]["ecdf"]) == 2
    assert merged["correct"]["right_censored_count"] == 1
    assert merged["correct"]["ecdf"][-1]["fraction"] < 1.0


def test_labelled_histograms_share_edges_and_report_overlap():
    value = row("Bear", 10)
    dist = MOD._channel_distributions([value], "geometry_score", labelled=True)
    correct_edges = [(item["lo"], item["hi"]) for item in dist["correct"]["histogram"]]
    incorrect_edges = [(item["lo"], item["hi"]) for item in dist["incorrect"]["histogram"]]
    assert correct_edges == incorrect_edges
    assert dist["overlap_interval"] is None


def test_loader_rejects_partial_population_and_bad_reference_role(tmp_path):
    value = row("Bear", 10)
    source = tmp_path / "scores.jsonl"
    source.write_text(json.dumps(value) + "\n")
    MOD.load_jsonl(source, expected_candidate_count=2)
    value["reference_role"] = "typo"
    source.write_text(json.dumps(value) + "\n")
    try:
        MOD.load_jsonl(source, expected_candidate_count=2)
    except MOD.AnalysisInputError:
        pass
    else:
        raise AssertionError("invalid reference role was accepted")


def test_loader_rejects_corrupt_population_identity_and_geometry_selection(tmp_path):
    source = tmp_path / "scores.jsonl"
    first = row("Bear", 10)
    second = row("Dinosaur", 10)
    second["frame_index"] = 11
    source.write_text(json.dumps(first) + "\n" + json.dumps(second) + "\n")
    try:
        MOD.load_jsonl(source, expected_candidate_count=2)
    except MOD.AnalysisInputError as exc:
        assert "one-to-one" in str(exc)
    else:
        raise AssertionError("non-bijective stamp/frame mapping was accepted")

    for mutation in ("proposal", "histogram", "selection", "selection_tie"):
        value = row("Bear", 10)
        if mutation == "proposal":
            value["stage300"]["candidates"][1]["proposal6000_index"] = 0
        elif mutation == "histogram":
            value["stage6000"]["incorrect"]["bins"] = [5998]
            value["stage6000"]["incorrect"]["count"] = 5998
            value["stage6000"]["invalid_pose"]["bins"] = [1]
            value["stage6000"]["invalid_pose"]["count"] = 1
            value["stage6000"]["invalid_pose"]["edges"] = [0.0, 2.0]
        elif mutation == "selection":
            value["stage300"]["candidates"][1]["geometry_score"] = 1.0
        else:
            first, second = value["stage300"]["candidates"]
            first.update({"geometry_score": .9, "geometry_selected": False})
            second.update({"geometry_score": .9, "geometry_selected": True})
            value["stage300"]["candidates"] = [second, first]
            value["stage300"]["selected_index300"] = 1
        source.write_text(json.dumps(value) + "\n")
        try:
            MOD.load_jsonl(source, expected_candidate_count=2)
        except MOD.AnalysisInputError:
            pass
        else:
            raise AssertionError(f"corrupt {mutation} population was accepted")


def test_loader_rejects_labels_inside_gt_unavailable_population(tmp_path):
    value = row("Bear", 10)
    value["stage6000"].update({
        "gt_status": "unavailable", "correct": None,
        "incorrect": None, "invalid_pose": None,
    })
    for candidate_value in value["stage300"]["candidates"]:
        candidate_value.update({"gt_status": "unavailable", "correct": None})
    source = tmp_path / "scores.jsonl"
    source.write_text(json.dumps(value) + "\n")
    MOD.load_jsonl(source, expected_candidate_count=2)
    value["stage300"]["candidates"][1]["gt_status"] = "available"
    source.write_text(json.dumps(value) + "\n")
    try:
        MOD.load_jsonl(source, expected_candidate_count=2)
    except MOD.AnalysisInputError as exc:
        assert "GT-unavailable" in str(exc)
    else:
        raise AssertionError("label leaked into GT-unavailable population")


def test_analyzer_derives_reference_membership_from_pseudo_gt():
    value = row("Bear", 5, "evaluation_frame")
    pseudo = {"Bear": {"trusted": True, "reference_frames": [{"stamp_ns": 5}]}}
    try:
        MOD.analyze([value], pseudo, min_correct=1)
    except MOD.AnalysisInputError as exc:
        assert "reference role mismatch" in str(exc)
    else:
        raise AssertionError("reference-member mismatch was accepted")


def test_reusable_html_and_schema_are_written(tmp_path):
    rows, pseudo = fixture()
    source = tmp_path / "scores.jsonl"
    source.write_text("\n".join(json.dumps(value) for value in rows) + "\n")
    pseudo_path = tmp_path / "pseudo.json"
    pseudo_path.write_text(json.dumps(pseudo))
    result = MOD.analyze(MOD.load_jsonl(source, expected_candidate_count=2),
                         pseudo, min_correct=1)
    out = tmp_path / "report"
    MOD.write_report(result, out, source, pseudo_path)
    assert json.loads((out / "analysis.json").read_text())["schema_version"] == 1
    assert (out / "analysis-data.js").read_text().startswith("window.PEM_SCORE_ANALYSIS=")
    html = (out / "index.html").read_text()
    app = (out / "app.js").read_text()
    assert "analysis-data.js" in html
    assert "GT 신뢰 부족" in app
    assert "Weighted/composite score 없음" in app
    assert "threshold.sweep" in app
    assert "ecdfPlot" in app
    assert "postFilterStages" in app
