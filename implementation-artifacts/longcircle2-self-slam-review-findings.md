# Review triage: longcircle2 same-camera pseudo-GT

## Patched

- Required bag/metadata/trajectory and diagnostic-reference attestation; strict
  trajectory order/quaternion checks in every current consumer.
- Actual `--ros-setup` execution, saved-config validation, future setup hash.
- Structured per-observation quality rejection evidence and fixed-point final
  member gates.
- Full/reference-member/evaluation-only metric separation and non-independent
  wording.
- Invalid bbox handling, all-SLAM-missing object assignments, false stage O/X
  suppression, malformed provenance, duplicate reference poses, path-component
  symlinks, RGB-D parameter/duplicate/unmatched checks.

## Deferred hardening

- One-to-one/interpolated trajectory association and unique pose-reuse metrics.
  Current behavior is the approved nearest pose within 25 ms and reports coverage.
- A maximum-gap/contiguous-outage rejection threshold. Current acceptance freezes
  minimum coverage at 0.50 and reports the maximum gap; changing failure policy is
  an Ask First threshold decision.
- Atomic multi-file ORB/compact-reference publication and descriptor-based TOCTOU
  hardening. Current outputs are local, hash-pinned and fail closed on rerun.
- Full executable/container/shared-library attestation. Current provenance pins
  settings, vocabulary, launch config, workspace setup for future runs and all
  source/output hashes, but is not a reproducible container manifest.
- Two-pass support for bags whose messages are grouped entirely by topic. The
  validated target bag is interleaved and exact-stamp paired; queues remain bounded
  to avoid unbounded RGB-D memory use.

## Rejected as outside the approved intent

- Absolute bbox/sharpness floors: the frozen policy explicitly uses per-object
  joint quantiles q70 through q00 and requires asking before changing the quality
  gate.
- Exclusive global clustering or re-entry of observations: the frozen selection is
  the maximum *direct neighborhood of one selected center*, not a partition of all
  hypotheses. Serialized members remain center-direct and final-reference valid.
- Treating reference members as independent accuracy: explicitly prohibited and
  now separately reported.

No bad-spec or unresolved acceptance-criterion defect remains in these categories;
the deferred items are provenance/runtime hardening beyond the frozen feature.
