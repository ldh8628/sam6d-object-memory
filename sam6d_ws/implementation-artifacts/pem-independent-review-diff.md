# PEM independent verification — NO_VCS change manifest

`baseline_commit: NO_VCS`; a literal unified diff is unavailable. Review the following
story-owned changes as the best-effort diff scope.

## Runtime/model changes

- `run_inference_custom.py`: preserves the pre-random-sampling depth-valid/radius-inlier
  mask as `shape_mask[224,224]` and the yxyx crop for every PEM instance.
- `pose_estimation_model.py`: passes K/mask/crop and dense appearance inputs to an
  independent verifier; coarse selection is not delegated to appearance.
- `model_utils.py`: tracks the original 6,000 hypothesis index through top-300;
  `compute_coarse_Rt` always selects `scores.max(1)[1]`. It independently computes
  texture rank/score, projected bbox size ratio, point-splat IoU, coverage and color.
  Projection uses `object @ R.T + t`, 224 crop coordinates, chunked scatter and
  configurable dilation. No combined/z-score value is used by selection.
- `verify_config.py`: defaults to geometry-only plus diagnostic texture/strict-size/IoU
  channels. Texture and IoU thresholds default to `None`; strict size max is `1.0`.
- `sam6d_core.py` and `verify_eval.py`: propagate the new schema and deterministic dump
  settings. Verification ON/OFF smoke run produced exact final pose parity for 6/6 rows.

## Report/UI changes

- `build_pem_explorer.py`: validates/decodes observed-mask RLE, renders input/projected/
  intersection evidence, supports legacy fields, and maps current candidates by geometry
  rank. Safe staged report replacement remains in use.
- `tools/pem_explorer/app.js`, `app.css`: remove combined score/rank from the visible UI;
  show geometry selection and independent texture/size/IoU cards and candidate columns.
- `output/pem_explorer/index.html`: existing landing continues to embed
  `longcircle2_sam/index.html`, now labelled geometry-only + independent verification.
- `tools/analyze_pem_independent_verification.py`: streams large JSONL and reports object/
  global texture-rank, size threshold, IoU and coverage distributions without GT claims.

## Tests and actual outputs

- Added selection-invariance, projection-invalid, size boundary, RLE and evidence tests;
  updated landing expectation. Full result: `82 passed`, JS syntax checks pass.
- Full SAM run: 2,130 frames, 2,979 detections, 100 candidates/detection.
- New diagnostic: `output/longcircle2_sam_independent_diagnostic/` (3.7GB).
- Existing report safely replaced: `output/pem_explorer/longcircle2_sam/` (3.9GB,
  180,870 assets). Chrome rendered representative frame 586.
- Analysis: legacy combined reranked away from geometry top-1 in 1,632/2,979;
  strict size failures 566; IoU below .30/.40/.50 = 706/1,411/2,556.
- Representative choco frame 586: size ratio 1.90158 FAIL, IoU .59794.
- Representative saffron frames: size passes, IoU .23514/.21048.
- SAM-camera GT remains unavailable; results are distributions, not accuracy.

## Files in review scope

`implementation-artifacts/spec-pem-size-iou-candidate-filter.md`,
`sam6d_master/SAM-6D/Pose_Estimation_Model/run_inference_custom.py`,
`sam6d_master/SAM-6D/Pose_Estimation_Model/model/pose_estimation_model.py`,
`sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py`,
`realtime/verify_config.py`, `realtime/sam6d_core.py`, `temp/verify_eval.py`,
`tools/build_pem_explorer.py`, `tools/pem_explorer/app.js`,
`tools/pem_explorer/app.css`, `tools/analyze_pem_independent_verification.py`,
`tests/test_pem_candidate_verification.py`, `tests/test_pem_explorer.py`, and
`output/pem_explorer/index.html`.
