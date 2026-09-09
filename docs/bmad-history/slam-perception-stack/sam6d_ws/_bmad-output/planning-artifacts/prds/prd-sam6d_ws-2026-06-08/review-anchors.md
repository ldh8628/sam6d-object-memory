# PRD Code-Anchor Accuracy Review

**PRD:** `_bmad-output/planning-artifacts/prds/prd-sam6d_ws-2026-06-08/prd.md`
**Project root:** `/home/ldh9501/temp_ws/CLI_environment/sam6d_ws`
**Reviewer:** technical accuracy reviewer (read-only)
**Date:** 2026-06-08

## Summary

All 11 anchors verified **ACCURATE**. All Deliverable files-to-modify exist at the stated paths. No corrections required. One minor characterization note (non-blocking) recorded for anchor `sam6d_inference_node.py:826-827`.

## Per-anchor findings

### 1. `detector.py:286-288` — semantic confidence_thresh gate — **ACCURATE**
File: `sam6d_master/SAM-6D/Instance_Segmentation_Model/model/detector.py`
Lines 286-288:
```
        idx_selected_proposals = torch.arange(
            len(score_per_proposal), device=score_per_proposal.device
        )[score_per_proposal > self.matching_config.confidence_thresh]
```
The semantic (per-proposal) score is gated by `confidence_thresh`. Matches the PRD claim. (PRD also cites `confidence_thresh=0.2` as the value; the threshold field reference is correct — the numeric 0.2 lives in matching config, not on these lines, but the gate location is accurate.)

### 2. `run_batch_inference_fast.py:544-548` — det_score_thresh filter — **ACCURATE**
File: `sam6d_master/SAM-6D/run_batch_inference_fast.py`
Lines 544-548:
```
    with open(seg_path) as f:
        dets_ = json.load(f)
    dets = [d for d in dets_ if d["score"] > det_score_thresh]
    if not dets:
        return None, None, None, None, None
```
ISM `score > det_score_thresh` filter, returns empty on zero detections. Matches PRD.

### 3. `run_batch_inference_fast.py:587-594` — np.random.choice observed-point subsample — **ACCURATE**
Lines 587-594:
```
        n = cfg.n_sample_observed_point
        idx = (
            np.random.choice(len(choose), n)
            if len(choose) <= n
            else np.random.choice(len(choose), n, replace=False)
        )
        choose = choose[idx]
        cloud  = cloud[idx]
```
Unseeded `np.random.choice` selecting the observed point subset per frame. Matches PRD.

### 4. `run_batch_inference_fast.py:680-689` — pose_scores = pred_pose_score * score written unconditionally — **ACCURATE**
Lines 680-689:
```
            pose_scores = (out["pred_pose_score"] * out["score"]).detach().cpu().numpy()
        else:
            pose_scores = out["score"].detach().cpu().numpy()
        pred_rot   = out["pred_R"].detach().cpu().numpy()
        pred_trans = out["pred_t"].detach().cpu().numpy() * 1000

        for idx, det in enumerate(detections):
            detections[idx]["score"] = float(pose_scores[idx])
            detections[idx]["R"]     = pred_rot[idx].tolist()
            detections[idx]["t"]     = pred_trans[idx].tolist()
```
`pose_scores = pred_pose_score * score` (line 680, when pred_pose_score present) and the result is written into every detection unconditionally (686-689) — no threshold comparison. Matches PRD claim that the score is recorded for all detections without any gating. Range is correct.

### 5. `model_utils.py:221` — torch.rand RANSAC pose hypothesis sampling — **ACCURATE**
File: `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py`
Line 221 (under `# sample pose hypothese`, 218):
```
    idx = torch.searchsorted(cumsum_weights, torch.rand(B, n_proposal1*3, device=device))
```
Unseeded `torch.rand` drives the inverse-CDF sampling of pose hypotheses in coarse matching. Matches PRD.

### 6. `model_utils.py:277-285` — pose_score formula — **ACCURATE**
Lines 277-285:
```
    # compute score
    pred_pts = (pts1 - pred_t.unsqueeze(1)) @ pred_R
    dis = torch.sqrt(pairwise_distance(pred_pts, model_pts)).min(2)[0]
    mask = (label1>0).float()
    pose_score = (dis < dis_thres).float()
    pose_score = (pose_score * mask).sum(1) / (mask.sum(1) + 1e-8)
    pose_score = pose_score * mask.mean(1)

    return pred_R, pred_t, pose_score
```
The `# compute score` block computes `pose_score` and returns it (285). Matches PRD. (PRD FR-2/FR-3 also cite the sub-range `:281-283` for the formula body; that sub-range falls correctly inside this block — `pose_score` assignments are at 281-283.)

### 7. `sam6d_inference_node.py:898` and `:921` — _publish_poses / _publish_pose_transforms — **ACCURATE**
File: `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py`
Line 898: `    def _publish_poses(self, poses: list[PoseDetection]) -> float:`
Line 921: `    def _publish_pose_transforms(`
Both publish-method definitions are at exactly the cited lines. Matches PRD.

### 8. `sam6d_inference_node.py:826-827` — no-detection handling path — **ACCURATE** (with characterization note)
Lines 826-827:
```
        if not pem_ret["ok"]:
            raise RuntimeError(f"PEM 실패: {pem_ret.get('error', '알 수 없는 오류')}")
```
This is the PEM-result `ok` failure branch. Verified cross-reference: in the backend, zero valid detections returns `{"ok": False, ..., "error": "no valid detection"}` (`run_batch_inference_fast.py:664-666`). Therefore the no-detection case currently terminates exactly at this branch (raises -> caught in `_trigger_cb`, no pose published). The lines and the control-flow role match the PRD.
Note (non-blocking): the branch is literally a generic PEM-failure/error handler, not a dedicated "no detection" branch; it currently conflates true PEM errors with the zero-detection case. The implementation step should be aware that "no detection" is signaled via `ok=False` + `error="no valid detection"`, not a distinct status. This does not invalidate the anchor.

### 9. `Pose_Estimation_Model/test_bop.py:200-201` — seed setting in original entry point — **ACCURATE**
File: `sam6d_master/SAM-6D/Pose_Estimation_Model/test_bop.py`
Lines 200-201:
```
    random.seed(cfg.rd_seed)
    torch.manual_seed(cfg.rd_seed)
```
Seed setting present in the original `__main__` entry point (193). Matches PRD claim that the original entry point seeds RNG (and that the live path omits it).

### 10. `config/base.yaml:102` — rd_seed: 1 — **ACCURATE**
File: `sam6d_master/SAM-6D/Pose_Estimation_Model/config/base.yaml`
Line 102: `rd_seed: 1`
Exact match.

## Deliverables — files-to-modify existence check

| Path | Exists |
|---|---|
| `sam6d_master/SAM-6D/run_batch_inference_fast.py` | YES |
| `src/sam6d_ros/sam6d_ros/sam6d_inference_node.py` | YES |
| `src/sam6d_ros/config/params.yaml` | YES |
| `src/sam6d_ros/config/no_cli_milk.yaml` | YES |
| `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py` (ref/non-modify) | YES |

All Deliverable paths resolve. (Note: built/installed copies of the node and configs also exist under `build/` and `install/`; the canonical source paths cited by the PRD are the `src/` ones, which is correct.)

## Verdict

**11 / 11 anchors ACCURATE. 0 OFF-BY-N. 0 WRONG. 0 NOT-FOUND.** All Deliverable files present. PRD anchors are safe to drive the implementation step. Only advisory note: anchor #8's branch is a generic PEM-failure handler that the backend also reuses for the zero-detection case (`error="no valid detection"`), which the implementer should keep in mind when adding the explicit no-object gate.
