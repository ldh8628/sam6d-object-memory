# Best-effort change bundle (baseline: NO_VCS)

The workspace has no VCS metadata or saved pre-change tree. This review input records the complete intended delta and the concrete files/functions changed; generated JSONL/JPEG artifacts are summarized rather than embedded.

## `_bmad` restoration

- Added `_bmad/scripts/resolve_customization.py`, restoring the current resolver behavior and adding `try: import tomllib / except ModuleNotFoundError: import tomli as tomllib` for Python 3.10.
- Added `_bmad/bmm/config.yaml`; `implementation_artifacts` remains `{project-root}/implementation-artifacts`.

## Diagnostic capture without selection changes

- `model_utils.py`: every full stage-300 analysis candidate now carries stable `rank_geo`, `R`, and physical `t_mm`.
- `sam6d_core.py`: pointwise evidence is serialized once under `verify`, not duplicated under `diagnostic`.
- `temp/verify_eval.py`: detection JSONL is streamed to a hidden temporary file, fsynced, and atomically replaced. Added `--skip-summary-report` for full evidence runs.
- A 2,979-row diagnostic was generated. `selection-parity.json` reports exact parity for ordered identity, selected proposal, R, and t_mm (all max differences 0).

## Shadow policy (`tools/build_pem_verification_policy.py`)

- Added Tune sweeps over rotations `[10,15,20,30]` degrees and translations `[25,50,100]` mm.
- A fallback search runs only when baseline Top-1 is withheld by `texture < 0.449562 AND mask_iou < 0.420998`.
- Candidates in the Top-1 cluster satisfy symmetry-aware SO(3) geodesic distance `<= rotation threshold AND Euclidean translation distance <= translation threshold`; they are excluded.
- The first remaining candidate in ascending original `rank_geo` passing `texture >= 0.449562 AND mask_iou >= 0.420998` is selected. No combined score is computed.
- Tune eligibility requires accepted accuracy >= 90% and correct-Top1-to-incorrect-replacement count 0. Maximum Tune coverage wins; exact ties choose smaller translation threshold, then larger rotation threshold.
- The 12-way sweep evaluates Tune records only. After the Tune winner is frozen, that one threshold is evaluated on Tune and Holdout for the published summary; sweep rows contain no Holdout fields.
- Sidecar contains selected thresholds, Tune/Holdout summaries, excluded cluster counts, no-candidate reasons, selected rank/proposal/R/t/channel scores/distances/GT, and `actual_pem_pose: unchanged`.
- Full stage-300 is the policy candidate scope. If a selected Shadow candidate is outside report ranks 0-99, its compact R/t/score record is materialized as a pinned report row with pose/mask proxy evidence.

## Full evidence report (`tools/build_pem_explorer.py`)

- Large pointwise arrays are spooled per detection and consumed/deleted during report construction; conflicting legacy copies fail closed.
- `render-topn >= 100` requires 50 GiB free before staging.
- For each candidate rank 0-99, builder creates pose projection, exact sampled geometry heatmap, exact sampled texture heatmap, rendered point-mask, and overlap. One input mask image is shared per detection.
- Projection failures create an annotated unavailable image/status/reason. Missing shape evidence also requires a reason.
- Staged validation checks rank-0-to-99 coverage, every asset link, exact geometry/texture evidence, projection reasons, and shape reasons before atomic publication. Existing completed reports are restored if publication fails.
- Shadow selections absent from the original top-100 are materialized from the signed policy fields and pinned; sidecar rank/proposal identity is checked before publication.
- Repeated provenance prose and duplicated `evidence.shape_metrics` are no longer serialized per candidate; the UI uses `candidate.shape` and static provenance text. This keeps `report-data.js` below the browser large-source limit.

## Explorer UI (`tools/pem_explorer/app.js`, `app.css`)

- Candidate identity is `rank_geo`, never array position.
- Query parameters `frame`, `object`, `rank_geo`, and `filter` initialize deep links; top-N expands to contain requested ranks.
- Withheld details show Top-1, excluded-similar count/cluster size, selected Shadow rank/proposal/scores/GT or a specific no-candidate reason, plus navigation to the pinned candidate row.
- Authoritative policy validation fails closed on schema, dataset, fingerprint, thresholds, decision math, locally recomputed Tune/Holdout split, cluster counts, selected score thresholds, distance outside cluster, and GT types. Boundary scores/distances retain source precision rather than rounded sidecar values.
- Projection/shape unavailable reasons are visible; the header shows Tune/Holdout fallback summaries.

## Tests and generated outputs

- Added/updated tests for inclusive pose-cluster boundaries, symmetry, duplicate ranks, original-order selection, both-channel threshold, no-candidate reasons, Tune/Holdout isolation, R/t in sidecar, spooling conflicts, 50 GiB preflight, atomic preservation, full evidence assets, Shadow rows outside top-100, rank deep links, and projection failures.
- Full suite: 153 passed.
- Published report: 2,130 frames, 2,979 detections, exactly 297,900 rank-0-to-99 candidates, 1,492,500 evidence files, 34 GiB. Independent scan found 0 rank gaps, 0 exact-evidence gaps, 0 shared-mask violations, and 0 missing/unrendered Shadow rows.
- Chrome deep link `?frame=62&object=Bear&rank_geo=42` rendered successfully and exposed proposal 3642, R/t, and all requested evidence links.

## Observed metrics

- Tune selected `20 degrees / 25 mm`: replacements 22/233 withheld, 20 correct, 0 correct-to-incorrect; accepted 420/466 correct (90.13%), coverage 68.83%.
- Frozen Holdout result: replacements 7/152 withheld, 7 correct; accepted 325/352 correct (92.33%), coverage 70.82%.
- The plan's estimated Holdout `10/9` is not reproduced under the specified distance rule. The three-count delta consists of saffron candidates whose SO(3) distance is 10.5-18.8 degrees and translation distance 6-20 mm, so they are inside the explicit 20-degree/25-mm AND cluster and must be excluded.
