# Best-effort change set (NO_VCS baseline)

There is no Git repository. This bundle enumerates the files and behavior changed
for the same-camera ORB-SLAM3 pseudo-GT feature.

## New files

- `configs/orbslam3_longcircle2_sam_rgbd.yaml`: exact 640x480 CameraInfo intrinsics,
  zero distortion, BGR, 16UC1 millimetre depth factor 1000, 2000 ORB features.
- `tools/run_orbslam3_rgbd_bag.py`: validates the bag, launches the installed ROS2
  ORB-SLAM3 RGB-D node, validates finite/monotonic TUM poses and normalized
  quaternions, computes nearest-stamp coverage, hashes inputs/outputs, and writes
  provenance. Refuses an unexpected dataset and an existing trajectory.
- `tools/extract_pose_reference.py`: atomically streams only stamp/index/object/
  score/R/t/bbox fields from a large PEM diagnostic JSONL.
- `pytest.ini`: limits collection to `tests/`.
- Unit tests for the runner and compact extractor.

## `tools/evaluate_slam_pose.py`

- Added SHA-256 and image-based crop-quality measurement. Each detection bbox is
  clipped to the frame and assigned bbox area plus grayscale Laplacian variance;
  missing images, invalid bbox, and empty crops get explicit rejection reasons.
- Added an adaptive joint quality gate. For each object it tries q70, q65, ...,
  q00 thresholds on both bbox area and crop sharpness, stopping at the first level
  with at least 10 selected records. It records the selected quantile, thresholds,
  counts, and rejection reasons. Fewer than 10 valid records is untrusted.
- Replaced transitive DBSCAN use in `build_pseudo_gt` with pairwise direct
  neighborhoods. Adjacency requires translation <= 0.08 m and symmetry-aware
  rotation <= 30 degrees. The center maximizes direct neighbor count; ties use
  lowest median normalized neighbor distance then earliest timestamp.
- Symmetric rotations are aligned to the center before rotation averaging.
  Translation uses the median. Members must be adjacent to the chosen center and
  are refined against the robust reference. Trust requires quality pass, at least
  5 members, and cluster share >= 0.50.
- Every object serializes trust/unavailable reason, quality gate, center/tie data,
  every reference-member stamp with residuals, and five representatives.
- CLI now requires `--frame-image-dir`; main enables the quality filter and writes
  `_provenance` (dataset, trajectory/reference hashes, frozen thresholds and
  self-consistency warning) plus `reference_assignments.json`.

## `tools/validate_rgbd_dataset.py` and manifest

- Added policy `same_camera_self_slam_reference`. It is the only pseudo-GT path
  allowed for a `dedicated_sam_pose_camera`; the camera remains related to the old
  SLAM camera as `distinct_unregistered_camera`.
- The policy requires source dataset id and optical frame equality. Trajectory,
  ORB settings, provenance and pseudo-GT paths must be nonempty relative regular
  files inside the bag, not direct symlinks, and match declared SHA-256 digests.
- Parsed provenance must be accepted and match dataset id, optical frame, bag DB
  filename, trajectory filename and digest.
- When pseudo-GT is supplied, both its path and trajectory path must be the exact
  manifest-attested files. `_provenance.dataset_id` and trajectory digest must
  match the policy.
- `data/longcircle2_sam/dataset_manifest.json` now enables this policy and pins
  `self_slam/{CameraTrajectory.txt,provenance.json,pseudo_ground_truth.json,
  orbslam3-settings.yaml}`. The missing external inter-camera transform remains
  documented and unused.

## `temp/verify_eval.py`

- Replaced Python-version-incompatible `rosbag2_py` frame loading with the existing
  pure-Python `rosbags.AnyReader`. The already validated canonical topics are
  deserialized with the ROS2 Humble typestore, exact header stamps are paired,
  stride/limit/count conditions are enforced, and pending colors fail closed.
- The inference/selection path is otherwise unchanged: geometry-only 300-to-1,
  independent texture/size/IoU diagnostics, fixed seed.

## HTML builder/UI

- `load_reference_inputs` retains `_provenance` and trusted/untrusted per-object
  cluster summaries in addition to validated trusted poses.
- Each detection receives one of `trusted_reference_member`,
  `untrusted_cluster_member`, or `evaluation_frame` based on its stamp and object.
- Report ground-truth kind is `same-camera ORB-SLAM3 pseudo-GT`; it exposes object
  quality/cluster summaries and warns reference members are not independent trials.
- UI cards/details show the frame reference role, GT trusted/untrusted reason,
  quality sample count, cluster size/share, while preserving axes, 6000/300 O/X,
  geometry/texture/shape scores and evidence images.
- Landing defaults to the same-camera report and links the preserved no-GT report.

## Generated and verified artifacts

- ORB trajectory: 1766 poses; 1779/2130 frames matched within 25 ms (83.521%);
  maximum gap 0.206 s; accepted provenance.
- Final pseudo-GT after raw-bag crop-quality hardening: trusted for Bear, Dinosaur,
  Mugcup_high, Sauce_high, Sikhye_high, milk, saffron. Febreze_high is untrusted
  because 9/20 is 45%; choco_hazelnut_high is untrusted at 4/15 (26.7%).
- Final report: 2,130 frames and 2,979 detections. Among 1,779 GT-available stage
  results, stage6000 O=1,516/X=263 and stage300 O=1,428/X=351. Untrusted,
  SLAM-missing and invalid-pose detections expose no false O/X.
- New default report: 3.9 GB, 180876 files, all four diagnostic capabilities true.
  Previous 3.9 GB no-GT report preserved at `longcircle2_sam_no_gt_20260823`.
- Headless Chrome rendered the report successfully. `python -m pytest -q` passes
  93 tests. The only warning is the pre-existing pynvml deprecation warning.

## Review remediation

- `evaluate_slam_pose.py` now requires a dataset manifest and accepted trajectory
  provenance, verifies bag/metadata/trajectory hashes, optical frame, explicit
  `T_wc` convention, strict timestamps and normalized quaternions, and requires a
  hash-attested sidecar for every compact reference input. Duplicate object/stamp
  rows and non-rigid reference transforms fail closed.
- `extract_pose_reference.py` can create the required sidecar binding dataset id,
  manifest, full diagnostic source hash, compact output hash/schema and row count.
- Quality assignments are structured per observation with measured area/sharpness,
  the selected thresholds, and explicit per-axis rejection reasons. Robust-member
  filtering is a remove-only fixed point so final serialized members satisfy the
  final reference gates.
- Metrics now report full self-consistency, reference-member, and evaluation-only
  cohorts separately. The final cohort excludes all 102 trusted reference members:
  1,734 poses, 57.44% rotation <=30 degrees and 57.04% pose <=30 degrees/10 cm.
  Misleading “Independent anchor agreement” wording was removed.
- The ORB runner sources the actual validated `--ros-setup`, hashes it in future
  provenance, and validate-only mode checks the saved launch config against current
  inputs. The HTML trajectory loader no longer sorts invalid input and checks
  quaternion normalization.
- RGB-D frame loading validates stride/limit, rejects duplicate stamps, bounds both
  pending streams, discards skipped-depth payloads, and fails on unmatched selected
  depth frames. Attested derivative validation rejects symlinks in any path component
  and malformed nested provenance structures with controlled errors.
- Quality imagery now comes directly from the attested bag instead of a mutable
  JPEG cache. This correctly changes Febreze_high to untrusted (9/20, 45%) and
  leaves seven trusted objects. The manifest-pinned pseudo-GT SHA-256 is
  `6beed2504e4b4bf0b698298c4e1b87bf9392fbc2b37521947ac334bd9df38706`.
  Full regression result is now 100 passed; the observed pynvml deprecation warning
  remains.
