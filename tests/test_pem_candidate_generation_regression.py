import sys
from pathlib import Path

import torch


ROOT = Path(__file__).resolve().parents[1]
PEM = ROOT / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
sys.path.insert(0, str(PEM / "model" / "pointnet2"))
sys.path.insert(0, str(PEM / "utils"))

import model_utils as module  # noqa: E402


def _legacy_generation(atten, pts1, pts2, nproposal1, nproposal2):
    """Frozen pre-filter 3-point WSVD and residual top-k implementation."""
    batch, n1, _ = pts1.size()
    n2 = pts2.size(1)
    pred_score = torch.softmax(atten, dim=2) * torch.softmax(atten, dim=1)
    label1 = torch.max(pred_score[:, 1:, :], dim=2)[1]
    label2 = torch.max(pred_score[:, :, 1:], dim=1)[1]
    weights1 = (label1 > 0).float()
    weights2 = (label2 > 0).float()
    pred_score = pred_score[:, 1:, 1:].contiguous()
    pred_score = pred_score * weights1.unsqueeze(2) * weights2.unsqueeze(1)
    pred_score = pred_score.reshape(batch, n1 * n2) ** 1.5
    cumsum = torch.cumsum(pred_score, dim=1)
    cumsum /= cumsum[:, -1].unsqueeze(1).contiguous() + 1e-8
    index = torch.searchsorted(cumsum, torch.rand(batch, nproposal1 * 3))
    index1, index2 = index.div(n2, rounding_mode="floor"), index % n2
    index1 = torch.clamp(index1, max=n1 - 1).unsqueeze(2).repeat(1, 1, 3)
    index2 = torch.clamp(index2, max=n2 - 1).unsqueeze(2).repeat(1, 1, 3)
    p1 = torch.gather(pts1, 1, index1).reshape(batch * nproposal1, 3, 3)
    p2 = torch.gather(pts2, 1, index2).reshape(batch * nproposal1, 3, 3)
    rotations, translations = module.WeightedProcrustes()(p2, p1, None)
    rotations = rotations.reshape(batch, nproposal1, 3, 3)
    translations = translations.reshape(batch, nproposal1, 1, 3)
    p1 = p1.reshape(batch, nproposal1, 3, 3)
    p2 = p2.reshape(batch, nproposal1, 3, 3)
    residual = torch.norm((p1 - translations) @ rotations - p2, dim=3).mean(2)
    proposal_ids = torch.topk(residual, nproposal2, dim=1, largest=False)[1]
    return residual, proposal_ids


def test_6000_and_top300_generation_values_are_legacy_exact(monkeypatch):
    torch.manual_seed(17)
    atten = torch.randn(1, 6, 7)
    pts1 = torch.randn(1, 5, 3)
    pts2 = torch.randn(1, 6, 3)
    rng_state = torch.random.get_rng_state()
    expected_residual, expected_ids = _legacy_generation(atten, pts1, pts2, 6000, 300)

    captured = {}
    def capture_stage6000(rs, ts, residual, appe, radius):
        captured["residual"] = residual.detach().clone()
        return [{}]

    monkeypatch.setattr(module, "_stage6000_score_analysis", capture_stage6000)

    def capture_stage300(rs, ts, scores, appe, info):
        captured["proposal_ids"] = appe["_proposal_ids"].detach().clone()
        return scores.max(1)[1]

    monkeypatch.setattr(module, "independent_candidate_verify", capture_stage300)
    torch.random.set_rng_state(rng_state)
    module.compute_coarse_Rt(
        atten, pts1, pts2, model_pts=pts2, n_proposal1=6000, n_proposal2=300,
        appe={"diagnostic": {"enabled": True, "score_analysis": {"enabled": True}},
              "radius": torch.ones(1)}, info={})
    assert torch.equal(captured["residual"], expected_residual)
    assert torch.equal(captured["proposal_ids"], expected_ids)
