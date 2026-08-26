"""ism_hsv.py — operational HSV hard-gate authority (Phase 1B: shadow only).

This is the SINGLE operational port of the validated research HSV gate
(`_ism_research_2026_07/yolo_localization_research/probes/eval_pipeline_end2end.py`,
mode "B"). Both operational entry points import it so the batch driver
(`yolo_ism_object_n.recognize_frame`) and the live ROS node
(`sam6d_multiobject_node` -> `recognize`) compute an IDENTICAL hsv score.

Authority (verbatim math, verified against the research code):
  reference : 42 render templates -> mask-interior pixels -> deterministic 20000-px
              sample (seed = view index) -> feat_rgb (Hue shift -3 in OpenCV units,
              i.e. round(-6/2); Sat x1.3; joint H(16)xS(8) hist; L1-normalized).
  query     : accepted box crop -> masked joint H(16)xS(8) hist; L1-normalized;
              NO hue/sat correction (only the reference is corrected).
  similarity: bc = sum(sqrt(q * proto)) per template;  sim = max_over_42(1 - sqrt(1 - bc)).
              higher = more similar. gate = sim >= threshold.

Phase 1B uses this in SHADOW only: the score is computed and logged; the final
accept/reject decision is NOT changed. There is no low-saturation exception
(the pinned mode-B code has none; matches the color-validation final decision).
"""
import hashlib
import os

import cv2
import numpy as np

# ---- frozen authority parameters (do not change without re-validating parity) ----
HSV_AUTHORITY_VERSION = "hsv-authority-1.0-modeB-2026-07-22"
HUE_SHIFT_DEG = -6            # 0-360 convention; applied to REFERENCE as round(-6/2) = -3 OpenCV units
SAT_GAIN = 1.3               # applied to REFERENCE only
HBINS, SBINS = 16, 8         # joint H x S histogram -> 128-D
SUB_N = 20000               # deterministic per-view pixel sample (reproduces research build)
MIN_MASK_PX = 50            # a render view with fewer mask pixels is skipped (research parity)


# =========================================================== core math (authority)
def feat_rgb(rgb_px):
    """RGB pixel list [N,3] uint8 -> hue/sat-corrected joint H x S histogram [128], L1.
    Verbatim port of eval_pipeline_end2end.feat_rgb (REFERENCE side)."""
    img = np.asarray(rgb_px, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]        # RGB->BGR
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + int(round(HUE_SHIFT_DEG / 2))) % 180        # -> -3, circular wrap
    hsv[..., 1] = np.clip(hsv[..., 1] * SAT_GAIN, 0, 255)
    x = cv2.calcHist([hsv.astype(np.uint8)], [0, 1], None,
                     [HBINS, SBINS], [0, 180, 0, 256]).flatten()
    return (x / max(x.sum(), 1e-12)).astype(np.float64)


def query_hist(bgr_crop, mask_crop):
    """Box crop (BGR) + its mask -> masked joint H x S histogram [128], L1. QUERY side.
    No hue/sat correction (matches research query path). mask_crop may be None/empty."""
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    m = None
    if mask_crop is not None:
        m = (np.asarray(mask_crop).astype(np.uint8) * 255)
        if int((m > 0).sum()) == 0:
            m = None
    x = cv2.calcHist([hsv], [0, 1], m, [HBINS, SBINS], [0, 180, 0, 256]).flatten()
    s = x.sum()
    return (x / s if s > 0 else x).astype(np.float64)


def similarity(q, proto):
    """q [128] L1 query, proto [V,128] L1 templates -> max Bhattacharyya-derived sim in [0,1].
    Verbatim port of eval_pipeline_end2end.hsv_of. Empty proto -> 1.0 (fail-open)."""
    if proto is None or len(proto) == 0:
        return 1.0
    bc = np.sqrt(np.maximum(q[None, :] * proto, 0.0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


# =========================================================== template cache
def _apply_hue_ocv(rgb_px, d_ocv):
    """Shift hue of RGB pixels by d_ocv OpenCV units (0-179 space), circular. Phase 1C
    class color correction: baked into a class's REFERENCE at cache-build time, so the
    runtime stays object-agnostic (no per-class branch at inference)."""
    if not d_ocv:
        return rgb_px
    hsv = cv2.cvtColor(np.asarray(rgb_px, np.uint8).reshape(-1, 1, 3)[:, :, ::-1],
                       cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + int(d_ocv)) % 180
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR).reshape(-1, 3)[:, ::-1]


def _template_signature(template_dir, hue_correction_ocv=0):
    """Content fingerprint of the 42 render rgb/mask/xyz files (change detection)."""
    h = hashlib.sha1()
    h.update(f"{HSV_AUTHORITY_VERSION}|{HUE_SHIFT_DEG}|{SAT_GAIN}|{HBINS}|{SBINS}|{SUB_N}"
             f"|hc{int(hue_correction_ocv)}".encode())
    for i in range(42):
        for pre in ("rgb", "mask", "xyz"):
            p = os.path.join(template_dir, f"{pre}_{i}.png" if pre != "xyz" else f"xyz_{i}.npy")
            if os.path.isfile(p):
                st = os.stat(p)
                h.update(f"{pre}{i}:{st.st_size}:{int(st.st_mtime)}".encode())
    return h.hexdigest()


def _sub(a, seed):
    """Deterministic 20000-px sample reproducing build_prototype_colors.sub()."""
    n = SUB_N
    if len(a) == 0:
        return np.zeros((0, 3), np.uint8)
    if len(a) <= n:
        if len(a) == n:
            return a
        r = np.random.RandomState(seed).randint(0, len(a), n - len(a))
        return np.concatenate([a, a[r]])
    return a[np.random.RandomState(seed).choice(len(a), n, replace=False)]


def build_reference(template_dir, hue_correction_ocv=0):
    """Build the reference histogram bank [V,128] from the render templates.

    Reproduces the research render42 extraction EXACTLY: a view is included only if
    rgb/mask/xyz all exist and mask has >= MIN_MASK_PX pixels; pixels are BGR->RGB,
    sampled to 20000 with seed = view index, then feat_rgb. -> parity with research PROTO.
    `hue_correction_ocv` (Phase 1C class color correction) shifts this class's reference hue
    by that many OpenCV units BEFORE feat_rgb (which then adds the shared base -3). 0 = none;
    e.g. Dinosaur = +9 (validated: real dinosaur reads greener than the render).
    Returns (proto [V,128] float64, n_view int).
    """
    feats = []
    for i in range(42):
        rp = os.path.join(template_dir, f"rgb_{i}.png")
        mp = os.path.join(template_dir, f"mask_{i}.png")
        xp = os.path.join(template_dir, f"xyz_{i}.npy")
        if not all(os.path.isfile(x) for x in (rp, mp, xp)):
            continue
        m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0
        bgr = cv2.imread(rp)
        if bgr is None or m.sum() < MIN_MASK_PX:
            continue
        px = _sub(bgr[m][:, ::-1], seed=i)          # BGR->RGB, deterministic 20000
        px = _apply_hue_ocv(px, hue_correction_ocv)  # Phase 1C class correction (0 = no-op)
        feats.append(feat_rgb(px))
    if not feats:
        return np.zeros((0, HBINS * SBINS), np.float64), 0
    return np.stack(feats), len(feats)


def cache_path_for(template_dir, feature_dir):
    name = os.path.basename(os.path.dirname(template_dir.rstrip("/"))) \
        if template_dir.rstrip("/").endswith("templates") else os.path.basename(template_dir)
    return os.path.join(feature_dir, f"{name}_hsv.npz")


def save_cache(path, proto, template_dir, n_view, hue_correction_ocv=0):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(path, proto=proto.astype(np.float64),
             signature=_template_signature(template_dir, hue_correction_ocv),
             authority_version=HSV_AUTHORITY_VERSION,
             hue_shift=HUE_SHIFT_DEG, sat_gain=SAT_GAIN,
             hbins=HBINS, sbins=SBINS, n_view=n_view, template_dir=template_dir,
             hue_correction_ocv=int(hue_correction_ocv))


def load_cache(path, template_dir, fail_open=True):
    """Load a template HSV cache. Returns (proto or None, meta dict).

    Validity: file exists AND signature matches the current template files AND authority
    version matches. On miss/corruption/mismatch: returns (None, {...}) — the caller's
    fail policy decides. `fail_open=True` documents that shadow mode must NOT alter the
    baseline decision on cache error (the runtime treats proto=None as sim=1.0 -> pass).
    Never auto-rebuilds at runtime (spec: no cache generation at operation start).
    """
    meta = {"path": path, "status": "ok", "authority_version": HSV_AUTHORITY_VERSION,
            "cache_version": "", "fail_open": fail_open}
    if not os.path.isfile(path):
        meta["status"] = "missing"
        return None, meta
    try:
        z = np.load(path, allow_pickle=True)
        sig = str(z["signature"])
        hc = int(z["hue_correction_ocv"]) if "hue_correction_ocv" in z else 0
        meta["cache_version"] = sig
        meta["hue_correction_ocv"] = hc
        if str(z["authority_version"]) != HSV_AUTHORITY_VERSION:
            meta["status"] = "authority-mismatch"
            return None, meta
        if sig != _template_signature(template_dir, hc):
            meta["status"] = "stale"
            return None, meta
        return z["proto"].astype(np.float64), meta
    except Exception as e:                                    # corrupt file
        meta["status"] = f"corrupt:{type(e).__name__}"
        return None, meta


# =========================================================== runtime shadow entry
def shadow_score(bgr, box, mask, proto):
    """Compute the shadow HSV score for one accepted candidate.

    bgr = full frame (BGR); box = (x1,y1,x2,y2); mask = full-frame bool/uint8 mask or None;
    proto = template bank [V,128] or None. Returns hsv_score in [0,1].
    proto None (cache miss) -> 1.0 (fail-open: never rejects in shadow simulation either).
    """
    x1, y1, x2, y2 = (int(v) for v in box)
    crop = bgr[y1:y2, x1:x2]
    if crop.size == 0:
        return 1.0
    mcrop = mask[y1:y2, x1:x2] if mask is not None else None
    q = query_hist(crop, mcrop)
    return similarity(q, proto)
