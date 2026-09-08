"""Bernoulli existence probability for object_memory landmarks.

A landmark's ``confidence`` is the probability r that the object actually
exists (IPDA / Bernoulli-filter track existence). Three update rules:

    missed, in view      r' = r(1 - P_D) / (1 - r * P_D)
    missed, out of view  P_D = 0 above  ->  r' = r  (no penalty)
    detected (accepted)  r' = r * P_D / (r * P_D + (1 - r) * c)

where P_D is the expected probability of detecting the object THIS frame
(0 when its landmark is outside the camera view) and c is the clutter ratio:
the relative likelihood that an accepted detection is a false alarm. Static
objects are assumed to survive (survival probability 1), so there is no
separate prediction step.

r is capped below 1 so that a miss always carries evidence (r = 1 is a fixed
point of the missed update).
"""

from __future__ import annotations

import math

R_MAX = 0.99
R_MIN = 1e-6


def update_existence_loglr(r: float, log_lr: float) -> float:
    """Bayes update of existence r in log-odds space: an accepted detection
    adds its log-likelihood-ratio to the current log-odds.

        logit(r') = logit(r) + log_lr

    This generalises ``update_existence_detected``: with
    ``log_lr = log(P_D / c)`` the two are identical (see the module test), but
    ``log_lr`` may instead be a per-detection, quality-weighted value so that a
    low-quality (e.g. degenerate-depth) detection carries little or negative
    evidence and cannot inflate r like a clean one. See core.features.
    """
    r = min(max(r, R_MIN), R_MAX)
    logit = math.log(r / (1.0 - r)) + log_lr
    r2 = 1.0 / (1.0 + math.exp(-logit))
    return min(max(r2, 0.0), R_MAX)


def update_existence_missed(r: float, p_d: float) -> float:
    """Bayes update of existence r when the object was NOT detected."""
    r = min(max(r, 0.0), R_MAX)
    p_d = min(max(p_d, 0.0), 1.0)
    return r * (1.0 - p_d) / (1.0 - r * p_d)


def update_existence_detected(r: float, p_d: float, clutter_ratio: float) -> float:
    """Bayes update of existence r when a detection was accepted."""
    r = min(max(r, 0.0), R_MAX)
    p_d = min(max(p_d, 0.0), 1.0)
    c = max(clutter_ratio, 1e-9)
    num = r * p_d
    return min(num / (num + (1.0 - r) * c), R_MAX)
