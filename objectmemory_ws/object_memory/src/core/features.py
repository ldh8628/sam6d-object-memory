"""Quality-weighted evidence for the existence filter (Story 5).

An accepted detection updates a landmark's existence r by adding a
log-likelihood-ratio (logLR) to its log-odds (see core.existence). Instead of a
constant logLR = log(P_D / c) for every detection, we weight the evidence by how
much the detection *looks like* a real object, using a few cheap, physically
interpretable features. The logistic form keeps log-odds a linear function of
features:

    logLR = log(P_D / c)                 # base: in-view detectability vs clutter
          + w_score * phi_score          # detector confidence, centred at 0.5
          + w_depth * phi_depth          # depth sanity: -1 if z is unphysical
          + w_resid * phi_resid          # agreement with the existing track

    phi_score = score - 0.5              # >0 supports real, <0 supports clutter
    phi_depth = -1 if z<z_min or z>z_max else 0   # z<0.05m ~= pose on the camera
    phi_resid = -clamp(resid_trans / gate, 0, 1)  # 0 for a fresh spawn (no track)

Backward compatibility: with all quality weights 0, logLR = log(P_D / c) and the
existence update reproduces the pre-Story-5 behaviour exactly.

Scope: this is EXISTENCE evidence ("is there really an object here"), not
identity ("is it really this class"); class mislabels are a separate axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# Physical constants (NOT data-tuned): a tabletop object cannot sit ~0 m from
# the camera, nor realistically past a few metres for these captures.
Z_MIN_M = 0.05
Z_MAX_M = 5.0

# Evidence weights. These are GLOBAL (identical for every object class); there
# are deliberately no per-class thresholds, so adding a new object needs no
# retuning.
#   w_score : detector confidence -> existence evidence.
#   w_depth : depth sanity -> a pose sitting on the camera is near-certain clutter.
#   w_resid : OFF by default. Within-gate residual is frame-to-frame pose JITTER
#             (estimator noise), NOT evidence about whether the object exists.
#             Penalising it made a jittery-but-real object (loop1 choco, pose
#             jitter to ~0.19 m) lose confidence and drop off the map. Jitter
#             belongs to pose FUSION (where it is averaged out) and gross
#             outliers are already handled by the association gate, so existence
#             ignores residual. Kept as an opt-in feature, not a default.
W_SCORE = 2.0
W_DEPTH = 4.0
W_RESID = 0.0
LOGLR_CLAMP = (-4.0, 2.5)   # a single detection can neither prove nor destroy


@dataclass(frozen=True)
class QualityWeights:
    w_score: float = W_SCORE
    w_depth: float = W_DEPTH
    w_resid: float = W_RESID
    z_min: float = Z_MIN_M
    z_max: float = Z_MAX_M
    clamp_lo: float = LOGLR_CLAMP[0]
    clamp_hi: float = LOGLR_CLAMP[1]

    @classmethod
    def legacy(cls) -> "QualityWeights":
        """All quality weights 0 -> logLR collapses to log(P_D / c)."""
        return cls(w_score=0.0, w_depth=0.0, w_resid=0.0)


def phi_score(score: float) -> float:
    return min(max(score, 0.0), 1.0) - 0.5


def phi_depth(z_cam: float, z_min: float = Z_MIN_M, z_max: float = Z_MAX_M) -> float:
    return -1.0 if (z_cam < z_min or z_cam > z_max) else 0.0


def phi_resid(resid_trans: Optional[float], gate: float) -> float:
    if resid_trans is None or gate <= 0.0:
        return 0.0
    return -min(max(resid_trans / gate, 0.0), 1.0)


def detection_log_lr(
    score: float,
    z_cam: float,
    resid_trans: Optional[float],
    gate: float,
    p_d: float,
    clutter_ratio: float,
    weights: QualityWeights = QualityWeights(),
) -> float:
    """Log-likelihood-ratio of one accepted detection (see module docstring)."""
    p_d = min(max(p_d, 1e-9), 1.0)
    c = max(clutter_ratio, 1e-9)
    base = math.log(p_d / c)
    log_lr = (
        base
        + weights.w_score * phi_score(score)
        + weights.w_depth * phi_depth(z_cam, weights.z_min, weights.z_max)
        + weights.w_resid * phi_resid(resid_trans, gate)
    )
    return min(max(log_lr, weights.clamp_lo), weights.clamp_hi)
