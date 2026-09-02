import math

import torch

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
import torch.nn as nn
from torch.nn import functional as F

import numpy as np

from pointnet2_utils import (
    gather_operation,
    furthest_point_sample,
)


class LayerNorm2d(nn.Module):
    def __init__(self, num_channels: int, eps: float = 1e-6) -> None:
        super().__init__()
        self.weight = nn.Parameter(torch.ones(num_channels))
        self.bias = nn.Parameter(torch.zeros(num_channels))
        self.eps = eps

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        u = x.mean(1, keepdim=True)
        s = (x - u).pow(2).mean(1, keepdim=True)
        x = (x - u) / torch.sqrt(s + self.eps)
        x = self.weight[:, None, None] * x + self.bias[:, None, None]
        return x


def interpolate_pos_embed(model, checkpoint_model):
    if 'pos_embed' in checkpoint_model:
        pos_embed_checkpoint = checkpoint_model['pos_embed']
        embedding_size = pos_embed_checkpoint.shape[-1]
        num_patches = model.patch_embed.num_patches
        num_extra_tokens = model.pos_embed.shape[-2] - num_patches
        # height (== width) for the checkpoint position embedding
        orig_size = int((pos_embed_checkpoint.shape[-2] - num_extra_tokens) ** 0.5)
        # height (== width) for the new position embedding
        new_size = int(num_patches ** 0.5)
        # class_token and dist_token are kept unchanged
        if orig_size != new_size:
            print("Position interpolate from %dx%d to %dx%d" % (orig_size, orig_size, new_size, new_size))
            extra_tokens = pos_embed_checkpoint[:, :num_extra_tokens]
            # only the position tokens are interpolated
            pos_tokens = pos_embed_checkpoint[:, num_extra_tokens:]
            pos_tokens = pos_tokens.reshape(-1, orig_size, orig_size, embedding_size).permute(0, 3, 1, 2)
            pos_tokens = torch.nn.functional.interpolate(
                pos_tokens, size=(new_size, new_size), mode='bicubic', align_corners=False)
            pos_tokens = pos_tokens.permute(0, 2, 3, 1).flatten(1, 2)
            new_pos_embed = torch.cat((extra_tokens, pos_tokens), dim=1)
            checkpoint_model['pos_embed'] = new_pos_embed



def sample_pts_feats(pts, feats, npoint=2048, return_index=False):
    '''
        pts: B*N*3
        feats: B*N*C
    '''
    sample_idx = furthest_point_sample(pts, npoint)
    pts = gather_operation(pts.transpose(1,2).contiguous(), sample_idx)
    pts = pts.transpose(1,2).contiguous()
    feats = gather_operation(feats.transpose(1,2).contiguous(), sample_idx)
    feats = feats.transpose(1,2).contiguous()
    if return_index:
        return pts, feats, sample_idx
    else:
        return pts, feats


def get_chosen_pixel_feats(img, choose):
    shape = img.size()
    if len(shape) == 3:
        pass
    elif len(shape) == 4:
        B, C, H, W = shape
        img = img.reshape(B, C, H*W)
    else:
        assert False

    choose = choose.unsqueeze(1).repeat(1, C, 1)
    x = torch.gather(img, 2, choose).contiguous()
    return x.transpose(1,2).contiguous()


def pairwise_distance(
    x: torch.Tensor, y: torch.Tensor, normalized: bool = False, channel_first: bool = False
) -> torch.Tensor:
    r"""Pairwise distance of two (batched) point clouds.

    Args:
        x (Tensor): (*, N, C) or (*, C, N)
        y (Tensor): (*, M, C) or (*, C, M)
        normalized (bool=False): if the points are normalized, we have "x2 + y2 = 1", so "d2 = 2 - 2xy".
        channel_first (bool=False): if True, the points shape is (*, C, N).

    Returns:
        dist: torch.Tensor (*, N, M)
    """
    if channel_first:
        channel_dim = -2
        xy = torch.matmul(x.transpose(-1, -2), y)  # [(*, C, N) -> (*, N, C)] x (*, C, M)
    else:
        channel_dim = -1
        xy = torch.matmul(x, y.transpose(-1, -2))  # (*, N, C) x [(*, M, C) -> (*, C, M)]
    if normalized:
        sq_distances = 2.0 - 2.0 * xy
    else:
        x2 = torch.sum(x ** 2, dim=channel_dim).unsqueeze(-1)  # (*, N, C) or (*, C, N) -> (*, N) -> (*, N, 1)
        y2 = torch.sum(y ** 2, dim=channel_dim).unsqueeze(-2)  # (*, M, C) or (*, C, M) -> (*, M) -> (*, 1, M)
        sq_distances = x2 - 2 * xy + y2
    sq_distances = sq_distances.clamp(min=0.0)
    return sq_distances


def compute_feature_similarity(feat1, feat2, type='cosine', temp=1.0, normalize_feat=True):
    r'''
    Args:
        feat1 (Tensor): (B, N, C)
        feat2 (Tensor): (B, M, C)

    Returns:
        atten_mat (Tensor): (B, N, M)
    '''
    if normalize_feat:
        feat1 = F.normalize(feat1, p=2, dim=2)
        feat2 = F.normalize(feat2, p=2, dim=2)

    if type == 'cosine':
        atten_mat = feat1 @ feat2.transpose(1,2)
    elif type == 'L2':
        atten_mat = torch.sqrt(pairwise_distance(feat1, feat2))
    else:
        assert False

    atten_mat = atten_mat / temp

    return atten_mat



def aug_pose_noise(gt_r, gt_t,
                std_rots=[15, 10, 5, 1.25, 1],
                max_rot=45,
                sel_std_trans=[0.2, 0.2, 0.2],
                max_trans=0.8):

    B = gt_r.size(0)
    device = gt_r.device

    std_rot = np.random.choice(std_rots)
    angles = torch.normal(mean=0, std=std_rot, size=(B, 3)).to(device=device)
    angles = angles.clamp(min=-max_rot, max=max_rot)
    ones = gt_r.new(B, 1, 1).zero_() + 1
    zeros = gt_r.new(B, 1, 1).zero_()
    a1 = angles[:,0].reshape(B, 1, 1) * np.pi / 180.0
    a1 = torch.cat(
        [torch.cat([torch.cos(a1), -torch.sin(a1), zeros], dim=2),
        torch.cat([torch.sin(a1), torch.cos(a1), zeros], dim=2),
        torch.cat([zeros, zeros, ones], dim=2)], dim=1
    )
    a2 = angles[:,1].reshape(B, 1, 1) * np.pi / 180.0
    a2 = torch.cat(
        [torch.cat([ones, zeros, zeros], dim=2),
        torch.cat([zeros, torch.cos(a2), -torch.sin(a2)], dim=2),
        torch.cat([zeros, torch.sin(a2), torch.cos(a2)], dim=2)], dim=1
    )
    a3 = angles[:,2].reshape(B, 1, 1) * np.pi / 180.0
    a3 = torch.cat(
        [torch.cat([torch.cos(a3), zeros, torch.sin(a3)], dim=2),
        torch.cat([zeros, ones, zeros], dim=2),
        torch.cat([-torch.sin(a3), zeros, torch.cos(a3)], dim=2)], dim=1
    )
    rand_rot = a1 @ a2 @ a3

    rand_trans = torch.normal(
        mean=torch.zeros([B, 3]).to(device),
        std=torch.tensor(sel_std_trans, device=device).view(1, 3),
    )
    rand_trans = torch.clamp(rand_trans, min=-max_trans, max=max_trans)

    rand_rot = gt_r @ rand_rot
    rand_trans = gt_t + rand_trans
    rand_trans[:,2] = torch.clamp(rand_trans[:,2], min=1e-6)

    return rand_rot.detach(), rand_trans.detach()


# ===========================================================================
# Stage V — 후보 텍스처 검증 (2026-08-21)
# ===========================================================================
# 기하 점수로 후보를 정렬한 뒤, **입력 이미지의 텍스처**와 **후보 포즈가 주장하는
# 모델 표면의 텍스처**를 맞대어 그 포즈가 뒤집혔는지 판정한다. 판정에서 그치지 않고
# 텍스처가 지지하는 후보로 승자를 교정하고, 근거(마진·신뢰도)를 함께 돌려준다.
#
# 왜 필요한가: 현행 승자 선택은 관측점↔모델 표면의 **거리만** 본다. 앞뒤가 뒤집힌
# 후보가 깊이를 똑같이 잘 설명하면 동점이 되고, 가설 추출 난수(torch.rand)가 승자를
# 정해 버린다. fine 정제는 init 을 중앙 7~15° 만 바꾸므로 이 뒤집힘을 되돌리지 못한다.
#
# 텍스처는 두 채널을 같은 최근접 대응 위에서 잰다(대응은 한 번만 계산하므로 둘째
# 채널의 추가 비용은 사실상 0):
#   s_feat : PEM 자체 ViT 특징 코사인 (dense_fm ↔ dense_fo). 조명·도메인갭에 강하다.
#   s_col  : 입력 crop 의 화소색 ↔ 템플릿 렌더의 점별 색. 인쇄무늬 물체(choco·milk·
#            Sikhye)의 앞뒤를 가르는 단서다. 비교 전에 **인스턴스별 화이트닝**을 해서
#            렌더↔실사의 전역 이득·색조 편향을 상쇄하고 '무늬 배치'만 남긴다.
# 점별 변별력 가중은 하지 않는다 — 실측에서 가중 없음과 동일하거나 악화였다
# (변별력이 높은 부위가 실사에서 가장 뭉개진다).

_SYM_CACHE = {}


def _sym_group(axis, step_deg, device, dtype):
    """선언된 대칭축 둘레의 동치 회전 [G,3,3]. 축이 없으면 단위행렬 하나뿐이다.

    표는 자동 유도하지 않는다 — 소각(10~20°) 대칭이 절대임계를 통과해 곰·공룡에
    가짜 대칭을 만들었던 전례가 있어, 손으로 쓴 보수적 목록만 config 로 받는다.
    """
    key = (tuple(axis) if axis else None, step_deg, str(device), str(dtype))
    if key in _SYM_CACHE:
        return _SYM_CACHE[key]
    mats = [torch.eye(3, device=device, dtype=dtype)]
    if axis:
        n = math.sqrt(sum(float(c) * float(c) for c in axis)) or 1.0
        x, y, z = (float(c) / n for c in axis)
        for d in range(step_deg, 360, step_deg):
            a = math.radians(d)
            c, s, k = math.cos(a), math.sin(a), 1.0 - math.cos(a)
            mats.append(torch.tensor(
                [[c + x * x * k, x * y * k - z * s, x * z * k + y * s],
                 [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
                 [z * x * k - y * s, z * y * k + x * s, c + z * z * k]],
                device=device, dtype=dtype))
    g = torch.stack(mats)
    _SYM_CACHE[key] = g
    return g


def _pairwise_angle_deg(R, sym):
    """후보 회전 R [K,3,3] 사이의 상대각 [K,K]. 대칭이 선언되면 동치 중 최소각.

    포즈는 물체→카메라 이므로 동치 자세는 R·S 다(fusion.align_to_symmetry 와 같은 규약).
    """
    rel = torch.einsum('iba,jbc->ijac', R, R)              # R_i^T R_j
    tr = torch.einsum('ijac,gca->ijg', rel, sym).max(2)[0]  # max_S trace(rel @ S)
    cos = ((tr - 1.0) * 0.5).clamp(-1.0, 1.0)
    return torch.rad2deg(torch.arccos(cos))


def _whiten(c):
    """[N,3] 색을 그 점 집합 안에서 채널별 평균 0 / 분산 1 로."""
    return (c - c.mean(0, keepdim=True)) / (c.std(0, keepdim=True) + 1e-6)


def _count_modes(ang, order, tol, elig, max_modes=8):
    """점수 내림차순으로 훑으며 tol 안에 안 붙는 후보마다 새 봉우리를 연다."""
    reps = []
    for k in order:
        if not elig[k]:
            continue
        if all(ang[k, r] > tol for r in reps):
            reps.append(k)
            if len(reps) >= max_modes:
                break
    return len(reps)


# ===========================================================================
# ch1 — 마스크 비교 게이트 (2026-08-24)
# ===========================================================================
# 기하 점수로 정렬한 뒤·텍스처 검증 앞에서, 후보 자세로 CAD 를 역투영해 만든 실루엣과
# ISM 마스크를 맞대어 **후보 자격만** 깎는다. 점수에 항을 더하지 않는다 — 색을 점수로
# 더했다가 FP 가 3~4 배 터지고 하드 게이트로 바꿔 −61% 를 얻은 전례(HSV)를 따른다.
#
# 단일 IoU 로는 두 가지를 못 가른다. 그래서 상보적인 항을 따로 둔다:
#   cov    = |Mo∩Mr|/|Mo|   렌더가 마스크를 설명하는가 (가림·잘림·depth 구멍에 강건)
#   prec   = |Mo∩Mr|/|Mr|   렌더가 마스크 밖으로 새는가
#   r_area = |Mr|/|Mo|      크기 정합 — '초코로 잘못 인식된 갈색 박스'(렌더가 확연히 작다)
#   dtheta = 주축 각도차     방향·형상 — 'saffron 축 어긋남'
#
# ★ 가림과 오인식은 r_area 를 **반대 방향**으로 민다: 가림은 Mo 를 줄이므로 ↑,
#   더 큰 엉뚱한 물체 위에 얹힌 오인식은 ↓. 그래서 r_area 는 **하한만** 건다.
#   양쪽을 거는 순간 가림에서 무너진다.
# ★ dtheta 는 실루엣이 길쭉할 때만 의미가 있다(주축은 180° 주기이고, 둥근 실루엣에서는
#   잡음이다). 관측 마스크 이심률이 ecc_min 미만이면 그 항을 쓰지 않는다.
#
# 용도가 둘이고, **문턱의 성격이 다르다**:
#   (A) 후보 게이트 — 상대. 300 개 중 잘못된 자세를 거른다. 가림 정도·스플랫의 계통적
#       과소피복·CAD 스케일 오차가 후보 전체에 공통으로 걸려 상쇄되므로 상대여야 한다.
#   (B) ISM 오인식 판별 — 절대. 오인식이면 후보 300 개가 **전부** 안 맞으므로 (A) 의
#       상대 문턱은 그 차이를 상쇄해 버린다. 그래서 S_best 를 따로 본다.
#       기본은 표시만 하고 거절하지 않는다(검출을 지우는 판단이라 실측 보정이 먼저다).


def _mask_moments(m):
    """이진 마스크 [.., G, G] 의 2차 모멘트 → (주축각 rad, 이심률).

    이심률 = sqrt(λmax/λmin). 둥근 실루엣이면 1 에 가깝고 그때 주축각은 잡음이다.
    """
    G = m.shape[-1]
    dev, dt = m.device, torch.float32
    idx = torch.arange(G, device=dev, dtype=dt)
    w = m.to(dt)
    n = w.flatten(-2).sum(-1).clamp(min=1.0)
    yy = idx.view(-1, 1).expand(G, G)
    xx = idx.view(1, -1).expand(G, G)
    my = (w * yy).flatten(-2).sum(-1) / n
    mx = (w * xx).flatten(-2).sum(-1) / n
    dy = yy - my[..., None, None]
    dx = xx - mx[..., None, None]
    cyy = (w * dy * dy).flatten(-2).sum(-1) / n
    cxx = (w * dx * dx).flatten(-2).sum(-1) / n
    cxy = (w * dx * dy).flatten(-2).sum(-1) / n
    # 2x2 대칭행렬의 고유값
    tr, det = cxx + cyy, cxx * cyy - cxy * cxy
    disc = torch.sqrt((tr * tr * 0.25 - det).clamp(min=0.0))
    lmax = tr * 0.5 + disc
    lmin = (tr * 0.5 - disc).clamp(min=1e-6)
    theta = 0.5 * torch.atan2(2.0 * cxy, cxx - cyy)      # 주축, 180° 주기
    return theta, torch.sqrt(lmax / lmin)


def _angdiff_pi(a, b):
    """180° 주기 각도차 [0, 90°] (라디안). 주축에는 앞뒤 구분이 없다."""
    d = (a - b) % math.pi
    return torch.minimum(d, math.pi - d)


def mask_compare_gate(pred_rs, pred_ts, model_pts, geo_scores, mg):
    """후보 자세로 만든 실루엣과 ISM 마스크를 비교해 자격을 깎는다.

    pred_ts 는 [B,P,3] 이다(호출부에서 squeeze(2) — appearance_rerank 와 같은 규약).
    mg = dict(설정..., mask=[B,Me,Me] 점유율, bbox=[B,4] y1y2x1x2, K=[B,3,3], radius=[B])

    되돌림: (keep1 [B,P] bool, stat dict) — stat 의 [B,P] 항은 채점 안 한 후보에 -1.
    """
    B, P = geo_scores.shape
    dev = geo_scores.device
    G = int(mg.get('grid', 32))
    Kn = int(min(mg.get('topk', 100), P))
    dil = int(mg.get('dilate', 1))
    z_min = float(mg.get('z_min', 1e-3))
    occ = float(mg.get('mask_occ', 0.0))
    terms = str(mg.get('score_terms', 'cov,prec,shape'))
    w_p = float(mg.get('w_p', 0.5))
    theta0 = math.radians(float(mg.get('theta0', 30.0)))
    ecc_min = float(mg.get('ecc_min', 1.6))
    guard = float(mg.get('gate_guard', 0.85))
    min_keep = int(mg.get('min_keep', 5))
    min_px = float(mg.get('min_bbox_px', 24))

    order = torch.argsort(geo_scores, dim=1, descending=True)[:, :Kn]        # [B,Kn]
    R = torch.gather(pred_rs, 1, order[:, :, None, None].expand(-1, -1, 3, 3))
    t = torch.gather(pred_ts, 1, order[:, :, None].expand(-1, -1, 3)).unsqueeze(2)  # [B,Kn,1,3]

    # 관측 마스크를 격자로. max-pool 이다 — 렌더가 스플랫의 **합집합**이라 "조금이라도
    # 덮였으면 켜짐"이 맞는 대응 규칙이다.
    Mo = F.adaptive_max_pool2d(mg['mask'].unsqueeze(1), (G, G)).squeeze(1) > occ   # [B,G,G]
    no_raw = Mo.flatten(1).sum(1)
    no = no_raw.clamp(min=1).to(torch.float32)

    # 모델 → 카메라. (pts1 - t) @ R = p_model 이므로 p_obs = p_model @ Rᵀ + t 이고,
    # dense_pm = pts / radius 였으므로 실제 미터는 × radius 다.
    pc = torch.matmul(model_pts.unsqueeze(1), R.transpose(-1, -2)) + t       # [B,Kn,M,3]
    pc = pc * mg['radius'].view(-1, 1, 1, 1)
    z = pc[..., 2]
    ok = z > z_min                       # 카메라 뒤/영점은 버린다
    zc = z.clamp(min=z_min)              # ⚠ 나누기 **전에** 클램프 — NaN/Inf 를 만들지 않는다
    Kc = mg['K']
    u = Kc[:, 0, 0].view(-1, 1, 1) * pc[..., 0] / zc + Kc[:, 0, 2].view(-1, 1, 1)
    v = Kc[:, 1, 1].view(-1, 1, 1) * pc[..., 1] / zc + Kc[:, 1, 2].view(-1, 1, 1)

    bb = mg['bbox']
    y1, y2, x1, x2 = bb[:, 0], bb[:, 1], bb[:, 2], bb[:, 3]
    sy = (G / (y2 - y1).clamp(min=1)).view(-1, 1, 1)
    sx = (G / (x2 - x1).clamp(min=1)).view(-1, 1, 1)
    gy = torch.floor((v - y1.view(-1, 1, 1)) * sy).long()
    gx = torch.floor((u - x1.view(-1, 1, 1)) * sx).long()
    ok = ok & (gy >= 0) & (gy < G) & (gx >= 0) & (gx < G)

    # ⚠ scatter_ 가 아니라 scatter_add_ 여야 한다 — scatter_ 는 무효 점의 0 이 유효 점의
    #   1 을 덮어쓴다. 더하는 값이 전부 정확히 1.0 이고 개수가 2²⁴ 미만이라 원자적 덧셈
    #   순서가 달라도 합이 정확하고 `>0` 은 비트 재현된다.
    flat = gy.clamp(0, G - 1) * G + gx.clamp(0, G - 1)
    acc = torch.zeros(B, Kn, G * G, device=dev, dtype=torch.float32)
    acc.scatter_add_(2, flat, ok.to(torch.float32))
    Mr = (acc > 0).view(B, Kn, G, G)
    for _ in range(max(dil, 0)):
        Mr = F.max_pool2d(Mr.to(torch.float32).view(B * Kn, 1, G, G), 3, 1, 1).view(B, Kn, G, G) > 0

    inter = (Mr & Mo.unsqueeze(1)).flatten(2).sum(2).to(torch.float32)
    nr = Mr.flatten(2).sum(2).to(torch.float32)
    cov = inter / no.view(-1, 1)
    prec = inter / nr.clamp(min=1.0)
    iou = inter / (no.view(-1, 1) + nr - inter).clamp(min=1.0)
    r_area = nr / no.view(-1, 1)

    th_o, ecc_o = _mask_moments(Mo)                       # [B]
    th_r, _ = _mask_moments(Mr)                           # [B,Kn]
    dtheta = _angdiff_pi(th_r, th_o.view(-1, 1))          # [B,Kn] 라디안
    # 길쭉하지 않으면 주축이 잡음이다 → 그 인스턴스는 shape 항을 쓰지 않는다.
    use_sh = (ecc_o >= ecc_min).view(-1, 1) & (nr > 0)
    shape = torch.where(use_sh, torch.exp(-dtheta / theta0), torch.ones_like(dtheta))

    S = cov.clone()
    if 'prec' in terms:
        S = S * prec.clamp(min=1e-6) ** w_p
    if 'shape' in terms:
        S = S * shape

    # ── (A) 후보 게이트 — 상대 ──────────────────────────────────────────
    smax = S.max(1, keepdim=True)[0]
    keep_k = S >= guard * smax
    if min_keep > 0:
        keep_k.scatter_(1, S.topk(min(min_keep, Kn), 1)[1], True)
    side = torch.minimum(y2 - y1, x2 - x1)
    dead = (smax.squeeze(1) <= 0) | (no_raw < 1) | (side < min_px)   # 게이트 해제
    keep_k = keep_k | dead.view(-1, 1)

    keep = torch.ones(B, P, dtype=torch.bool, device=dev)     # 채점 안 한 후보는 통과
    keep.scatter_(1, order, keep_k)
    keep.scatter_(1, geo_scores.argmax(1, keepdim=True), True)   # 기하 1등은 언제나

    def _full(x, fill=-1.0):
        f = torch.full((B, P), fill, device=dev, dtype=torch.float32)
        f.scatter_(1, order, x)
        return f

    # (B) 판별 통계는 **cov + r_area** 다. S 에는 prec 항이 들어 있어 가림에 민감해서
    # (가림 = 렌더가 마스크 밖으로 나감 = prec 하락) 오인식과 섞인다. 실측:
    # 오인식(2 배 큰 박스) cov .33 / r_area .33  vs  가림(절반) cov .93 / r_area 1.71
    # — cov 는 깨끗이 갈리고 r_area 는 **반대 방향**으로 벌어진다.
    best = S.argmax(1, keepdim=True)
    stat = {
        'S': _full(S), 'cov': _full(cov), 'prec': _full(prec), 'iou': _full(iou),
        'r_area': _full(r_area), 'dtheta': _full(torch.rad2deg(dtheta)),
        'ecc': ecc_o, 'use_shape': (ecc_o >= ecc_min),
        'S_best': S.gather(1, best).squeeze(1),
        'cov_best': cov.gather(1, best).squeeze(1),
        'r_area_best': r_area.gather(1, best).squeeze(1),
        'n_keep': keep_k.sum(1), 'n_eval': Kn, 'grid': G, 'dead': dead,
        'order': order,
    }
    return keep, stat


def appearance_rerank(pred_rs, pred_ts, geo_scores, appe, info=None, gate=None):
    """기하 상위 K 개 후보를 텍스처로 검증하고 승자를 돌려준다.

    appe = dict(dense_pm, dense_fm, dense_po, dense_fo, topk, w_geo, stride,
                [dense_cm, dense_co, names, verify])

    verify 가 없거나 enabled=False 면 예전 동작(외형 재정렬)과 **수치적으로 동일**하다.
    켜면 후보들을 회전으로 군집해 뒤집힘 후보와의 마진을 재고 PASS / CORRECTED /
    AMBIGUOUS 를 판정해 info 에 담는다.

    측정(0807/185223, 기준자세 163건·현행 뒤집힘 25건): 뒤집힘(>45°) 15%→9%,
    15° 이내 52%→60%, 뒤집힌 것의 48~52% 회복, 멀쩡한 것 파손 1~2%.
    """
    B, P = geo_scores.shape
    topk = int(min(appe.get('topk', 50), P))
    w_geo = float(appe.get('w_geo', 0.5))
    stride = int(appe.get('stride', 4))

    ver = appe.get('verify') or {}
    on = bool(ver.get('enabled', False))
    have_col = ('dense_cm' in appe) and ('dense_co' in appe)
    # 색 자산이 없으면 조용히 특징 채널만 쓴다(fail-open) — 자산 미비로 죽지 않게.
    w_col = (float(ver.get('w_col', 0.0)) if (on and have_col) else 0.0)
    tol = float(ver.get('mode_tol_deg', 30.0))
    flip = float(ver.get('flip_deg', 45.0))
    tau_pass = float(ver.get('tau_pass', 0.8))
    tau_corr = float(ver.get('tau_corr', 1.0))
    tau_scale = float(ver.get('tau_scale', 1.0)) or 1.0
    # geo_guard=0 이면 걸러내지 않는다(= 파리티 검사용 설정).
    guard = float(ver.get('geo_guard', 0.0)) if on else 0.0
    sym_tab = ver.get('symmetry_axes') or {}
    sym_step = int(ver.get('sym_step_deg', 10))
    # 보고서용 후보 덤프(기본 0=안 함). 판정에는 영향이 없다.
    dump = int(ver.get('dump_topk', 0)) if on else 0
    dump_all = bool(ver.get('dump_quats', False)) and on
    radius = appe.get('radius')
    names = appe.get('names') or [None] * B

    order = torch.argsort(geo_scores, dim=1, descending=True)[:, :topk]      # [B,K]
    idx_out, rows = [], []
    for b in range(B):
        sel = order[b]
        R = pred_rs[b][sel]                                                  # [K,3,3]
        t = pred_ts[b][sel].reshape(-1, 1, 3)                                # [K,1,3]
        pm = appe['dense_pm'][b][::stride]                                   # [n,3] 결정적 표본
        fm = torch.nn.functional.normalize(appe['dense_fm'][b][::stride], dim=1)
        po = appe['dense_po'][b]
        fo = torch.nn.functional.normalize(appe['dense_fo'][b], dim=1)
        obj = torch.einsum('kji,knj->kni', R, pm.unsqueeze(0) - t)           # 관측점 -> 물체좌표계
        nn = torch.cdist(obj, po.unsqueeze(0).expand(obj.size(0), -1, -1)).argmin(2)
        s_feat = torch.einsum('nd,knd->kn', fm, fo[nn]).mean(1)              # [K]
        g = geo_scores[b][sel]
        zs = lambda x: (x - x.mean()) / (x.std() + 1e-9)
        s = zs(s_feat) + w_geo * zs(g)
        s_col = None
        if w_col:
            cm = _whiten(appe['dense_cm'][b][::stride])                      # [n,3]
            co = _whiten(appe['dense_co'][b])                                # [M,3]
            s_col = (cm.unsqueeze(0) * co[nn]).mean((1, 2))                  # [K]
            s = s + w_col * zs(s_col)

        # 게이트(ch1 마스크 ∪ ch2 지도)가 준 자격. gate 가 None 이면 아래 전부 예전 경로다.
        gk = None if gate is None else gate['keep'][b][sel]                  # [K] bool
        gc = None if gate is None else gate['ch'][b][sel]                    # 1=mask 2=map 3=both

        if not on:
            if gk is None:
                idx_out.append(sel[torch.argmax(s)])       # ← 예전과 완전히 같은 연산
            else:
                e = gk.clone(); e[0] = True                # 기하 1등은 언제나 후보
                w = int(torch.argmax(s.masked_fill(~e, float('-inf'))))
                idx_out.append(sel[w])
                if info is not None:
                    info.setdefault('gate_aux', {})[b] = {
                        'win': w, 'win_ng': int(torch.argmax(s)),
                        'n_elig': int(e.sum()),
                        'admit': _ADMIT[int(gc[w])]}
            continue

        # 승자는 검증이 꺼져 있을 때와 **같은 연산**(torch.argmax)으로 뽑는다. 거의 같은
        # 점수의 후보가 흔해서(동점 구간이 이 단계의 본질이다) 동점 처리 규칙이 다른
        # numpy 로 옮기면 이유 없는 차이가 생길 수 있다. 파리티는 실측으로 확인했다:
        # w_col=0 · geo_guard=0 이면 검출 2075건의 R·t·score 가 OFF 와 완전히 같다.
        if guard > 0:
            elig_t = g >= guard * g.max()
        else:
            elig_t = torch.ones_like(g, dtype=torch.bool)
        if gk is not None:
            elig_ng = elig_t.clone(); elig_ng[0] = True    # 게이트 없었을 때의 자격
            elig_t = elig_t & gk
        elig_t[0] = True                                  # 기하 1등은 언제나 후보
        if guard > 0 or gk is not None:
            win = int(torch.argmax(s.masked_fill(~elig_t, float('-inf'))))
        else:
            win = int(torch.argmax(s))                    # ← 예전과 완전히 같은 연산
        if gk is not None and info is not None:
            info.setdefault('gate_aux', {})[b] = {
                'win': win,
                'win_ng': int(torch.argmax(s.masked_fill(~elig_ng, float('-inf')))),
                'n_elig': int(elig_t.sum()),
                'admit': _ADMIT[int(gc[win])]}

        # --- 나머지 판정은 작은 배열이라 CPU 에서 한다(동기화 1회) -------------
        sym = _sym_group(sym_tab.get(names[b]), sym_step, R.device, R.dtype)
        ang = _pairwise_angle_deg(R, sym).detach().cpu().numpy()
        s_np = s.detach().cpu().numpy()
        g_np = g.detach().cpu().numpy()
        elig = elig_t.detach().cpu().numpy()
        s_e = np.where(elig, s_np, -np.inf)
        far = (ang[win] > flip) & elig
        margin = float(s_np[win] - s_np[far].max()) if bool(far.any()) else float('inf')
        same = bool(ang[win, 0] <= tol)                   # sel 은 기하 내림차순 → 0 이 기하 1등
        if same and margin >= tau_pass:
            verdict = 'PASS'
        elif (not same) and margin >= tau_corr:
            verdict = 'CORRECTED'
        else:
            verdict = 'AMBIGUOUS'
        conf = 1.0 if margin == float('inf') else 1.0 / (1.0 + math.exp(-margin / tau_scale))

        idx_out.append(sel[win])
        cands = None
        if dump and info is not None:
            # 기하 상위 M 과 텍스처 재정렬 상위 M 의 합집합. 두 패널을 같은 목록에서
            # 그리기 위한 것이라 판정에 관여하지 않는다.
            rg = np.argsort(-g_np)
            rs = np.argsort(-s_e)
            keep = sorted(set(rg[:dump].tolist()) | set(rs[:dump].tolist()) | {0, win})
            rank_g = {int(k): int(i) for i, k in enumerate(rg)}
            rank_s = {int(k): int(i) for i, k in enumerate(rs)}
            sc = 1.0 if radius is None else float(radius[b])
            Rk = R.detach().cpu().numpy()
            tk = (pred_ts[b][sel] * sc).detach().cpu().numpy()
            scol = None if s_col is None else s_col.detach().cpu().numpy()
            cands = [{
                'R': [[round(float(v), 6) for v in row] for row in Rk[k]],
                't_mm': [round(float(v) * 1000.0, 3) for v in tk[k]],
                'geo': round(float(g_np[k]), 5),
                's_feat': round(float(s_feat[k]), 5),
                's_col': (None if scol is None else round(float(scol[k]), 5)),
                's': round(float(s_np[k]), 5),
                'rank_geo': rank_g[int(k)], 'rank_s': rank_s[int(k)],
                'elig': bool(elig[k]),
            } for k in keep]
        quats = None
        if dump_all and info is not None:
            # shortlist 전체를 사원수로 압축해 남긴다(후보당 6수). '정답이 후보 안에
            # 있었는가 / 몇 등이었는가' 를 사후에 재기 위한 것이라 판정과 무관하다.
            Rq = R.detach().cpu().numpy()
            tr = Rq[:, 0, 0] + Rq[:, 1, 1] + Rq[:, 2, 2]
            qw = np.sqrt(np.maximum(0.0, 1.0 + tr)) / 2.0
            d = np.maximum(4.0 * qw, 1e-8)
            quats = [[round(float((Rq[k, 2, 1] - Rq[k, 1, 2]) / d[k]), 5),
                      round(float((Rq[k, 0, 2] - Rq[k, 2, 0]) / d[k]), 5),
                      round(float((Rq[k, 1, 0] - Rq[k, 0, 1]) / d[k]), 5),
                      round(float(qw[k]), 5),
                      round(float(g_np[k]), 4), round(float(s_np[k]), 4)]
                     for k in range(len(Rq))]
        if info is not None and gate is not None and gc is not None:
            # ── 후보 소실 케이스 보관 ─────────────────────────────────────
            # 게이트/텍스처가 정답 후보를 지웠는지는 **온라인에서 알 수 없다**(정답이
            # 없다). 그래서 판정하지 않고 shortlist 전체를 압축해 남긴다 — 나중에
            # SLAM 궤적 기준을 붙이면 "정답이 후보 안에 있었는데 어느 관문에서
            # 죽었는가"를 재추론 없이 되돌려 볼 수 있다.
            # ★ 마스크 항을 **전부** 남긴다. cov 만 남기면 결합 점수
            #   S = cov · prec^w_p · shape 를 사후에 재현할 수 없어 스윕이 무의미해진다
            #   (실측: cov 는 후보 대부분이 1.0 이라 단독으로는 아무것도 안 걸러진다).
            st = gate['stat']
            covb = st['cov'][b][sel].detach().cpu().numpy()
            prb = st['prec'][b][sel].detach().cpu().numpy()
            rab = st['r_area'][b][sel].detach().cpu().numpy()
            dtb = st['dtheta'][b][sel].detach().cpu().numpy()
            chb = gc.detach().cpu().numpy()
            Rq = R.detach().cpu().numpy()
            tr_ = Rq[:, 0, 0] + Rq[:, 1, 1] + Rq[:, 2, 2]
            qw = np.sqrt(np.maximum(0.0, 1.0 + tr_)) / 2.0
            dd = np.maximum(4.0 * qw, 1e-8)
            info.setdefault('gate_cases', {})[b] = {
                'win': win, 'win_ng': int(np.argmax(s_e)),
                'ecc': round(float(st['ecc'][b]), 3),
                'use_shape': bool(st['use_shape'][b]),
                # [qx qy qz qw geo s cov prec r_area dtheta ch]
                'q': [[round(float((Rq[k, 2, 1] - Rq[k, 1, 2]) / dd[k]), 4),
                       round(float((Rq[k, 0, 2] - Rq[k, 2, 0]) / dd[k]), 4),
                       round(float((Rq[k, 1, 0] - Rq[k, 0, 1]) / dd[k]), 4),
                       round(float(qw[k]), 4),
                       round(float(g_np[k]), 4), round(float(s_np[k]), 4),
                       round(float(covb[k]), 3), round(float(prb[k]), 3),
                       round(float(rab[k]), 3), round(float(dtb[k]), 1),
                       int(chb[k])] for k in range(len(Rq))],
            }

        if info is not None:
            rows.append({
                'verdict': verdict,
                'conf': round(conf, 4),
                'margin': (None if margin == float('inf') else round(margin, 4)),
                's_feat': round(float(s_feat[win]), 4),
                's_col': (round(float(s_col[win]), 4) if s_col is not None else None),
                'n_modes': _count_modes(ang, np.argsort(-s_e), tol, elig),
                'geo_rank_used': win,
                'geo_ratio': round(float(g_np[win] / (g_np[0] + 1e-12)), 4),
                **({'cands': cands} if cands else {}),
                **({'quats': quats} if quats else {}),
            })
    if info is not None and rows:
        info['verify'] = rows
    return torch.stack(idx_out)




# 승자를 들여보낸 채널. 0 = 어느 채널도 아님 = 기하 1등 fail-open 으로만 살아남음.
_ADMIT = {0: 'rank1', 1: 'mask', 2: 'map', 3: 'both'}


def map_prior_gate(pred_rs, mp):
    """ch2 — 등록된 Map 자세와 맞는 후보에게 자격을 준다(대칭 접힘).

    mp = dict(admit_deg, ref_R=[B,3,3], ref_ok=[B] bool, names, sym_axes, sym_step)

    각도 수학과 대칭표는 **새로 만들지 않는다** — _sym_group/_pairwise_angle_deg 가
    이미 R·S 동치 규약으로 구현돼 있고 verify_config 의 표를 그대로 먹는다.
    되돌림: keep2 [B,P] bool (등록 안 된 인스턴스는 전부 False = 추가 자격 없음).
    """
    B, P = pred_rs.shape[0], pred_rs.shape[1]
    keep2 = torch.zeros(B, P, dtype=torch.bool, device=pred_rs.device)
    ok = mp.get('ref_ok')
    if ok is None or not bool(ok.any()):
        return keep2
    admit = float(mp.get('admit_deg', 20.0))
    tab = mp.get('sym_axes') or {}
    step = int(mp.get('sym_step', 10))
    names = mp.get('names') or [None] * B
    ref = mp['ref_R']
    for b in range(B):
        if not bool(ok[b]):
            continue
        sym = _sym_group(tab.get(names[b]), step, pred_rs.device, pred_rs.dtype)
        rel = torch.einsum('ba,kbc->kac', ref[b], pred_rs[b])      # ref^T @ R_k
        tr = torch.einsum('kac,gca->kg', rel, sym).max(1)[0]       # max_S trace(rel @ S)
        ang = torch.rad2deg(torch.arccos(((tr - 1.0) * 0.5).clamp(-1.0, 1.0)))
        keep2[b] = ang <= admit
    return keep2


def _gate_rows(st, idx, geo_scores, aux, mg):
    """게이트 진단을 검출 행에 실을 dict 목록으로. **동기화는 한 번만** 한다.

    스칼라를 하나씩 .item() 으로 읽으면 인스턴스당 동기화가 여러 번 나므로,
    필요한 값을 [B,N] 하나로 모아 한 번에 CPU 로 옮긴다.
    """
    B = geo_scores.shape[0]
    pick = idx.reshape(-1, 1)
    g1 = geo_scores.argmax(1, keepdim=True)
    flag = float(mg.get('ism_flag_below', 0.45))
    ramin = float(mg.get('r_area_min', 0.5))
    blob = torch.cat([
        torch.gather(st['S'], 1, pick), torch.gather(st['cov'], 1, pick),
        torch.gather(st['prec'], 1, pick), torch.gather(st['iou'], 1, pick),
        torch.gather(st['r_area'], 1, pick), torch.gather(st['dtheta'], 1, pick),
        torch.gather(st['S'], 1, g1),
        st['S_best'].reshape(-1, 1), st['r_area_best'].reshape(-1, 1),
        st['cov_best'].reshape(-1, 1),
        st['ecc'].reshape(-1, 1), st['use_shape'].reshape(-1, 1).float(),
        st['n_keep'].reshape(-1, 1).float(), st['dead'].reshape(-1, 1).float(),
        st.get('n_map', torch.zeros_like(st['n_keep'])).reshape(-1, 1).float(),
    ], dim=1).detach().cpu().tolist()

    rows = []
    for b, r in enumerate(blob):
        a = aux.get(b, {})
        rows.append({
            'S': round(r[0], 4), 'cov': round(r[1], 4), 'prec': round(r[2], 4),
            'iou': round(r[3], 4), 'r_area': round(r[4], 4), 'dtheta': round(r[5], 2),
            'S_geo1': round(r[6], 4), 'S_best': round(r[7], 4),
            'r_area_best': round(r[8], 4), 'cov_best': round(r[9], 4),
            'ecc': round(r[10], 3), 'use_shape': bool(r[11]),
            # (B) ISM 오인식 판별 — 절대. **기본은 표시만 하고 거절하지 않는다.**
            # 근거는 cov_best 와 r_area_best 다(S 는 prec 때문에 가림에 민감해 섞인다).
            # r_area 는 **하한만** 건다 — 가림은 r_area 를 올리므로 상한을 걸면 무너진다.
            'ism_suspect': bool(r[9] < flag or r[8] < ramin),
            'n_eval': st['n_eval'], 'n_keep': int(r[12]), 'grid': st['grid'],
            'off': bool(r[13]), 'n_map': int(r[14]),
            'admit': a.get('admit', 'nogate'),
            'changed': bool(a['win'] != a['win_ng']) if 'win' in a else False,
            'n_elig': a.get('n_elig'),
        })
    return rows


def compute_coarse_Rt(
    atten,
    pts1,
    pts2,
    model_pts=None,
    n_proposal1=6000,
    n_proposal2=300,
    appe=None,
    info=None,
):
    WSVD = WeightedProcrustes()

    B, N1, _ = pts1.size()
    N2 = pts2.size(1)
    device = pts1.device

    if model_pts is None:
        model_pts = pts2
    expand_model_pts = model_pts.unsqueeze(1).repeat(1,n_proposal2,1,1).reshape(B*n_proposal2, -1, 3)

    # compute soft assignment matrix
    pred_score = torch.softmax(atten, dim=2) * torch.softmax(atten, dim=1)
    pred_label1 = torch.max(pred_score[:,1:,:], dim=2)[1]
    pred_label2 = torch.max(pred_score[:,:,1:], dim=1)[1]
    weights1 = (pred_label1>0).float()
    weights2 = (pred_label2>0).float()

    pred_score = pred_score[:, 1:, 1:].contiguous()
    pred_score = pred_score * weights1.unsqueeze(2) * weights2.unsqueeze(1)
    pred_score = pred_score.reshape(B, N1*N2) ** 1.5

    # sample pose hypothese
    cumsum_weights = torch.cumsum(pred_score, dim=1)
    cumsum_weights /= (cumsum_weights[:, -1].unsqueeze(1).contiguous()+1e-8)
    idx = torch.searchsorted(cumsum_weights, torch.rand(B, n_proposal1*3, device=device))
    idx1, idx2 = idx.div(N2, rounding_mode='floor'), idx % N2
    idx1 = torch.clamp(idx1, max=N1-1).unsqueeze(2).repeat(1,1,3)
    idx2 = torch.clamp(idx2, max=N2-1).unsqueeze(2).repeat(1,1,3)

    p1 = torch.gather(pts1, 1, idx1).reshape(B,n_proposal1,3,3).reshape(B*n_proposal1,3,3)
    p2 = torch.gather(pts2, 1, idx2).reshape(B,n_proposal1,3,3).reshape(B*n_proposal1,3,3)
    pred_rs, pred_ts = WSVD(p2, p1, None)
    pred_rs = pred_rs.reshape(B, n_proposal1, 3, 3)
    pred_ts = pred_ts.reshape(B, n_proposal1, 1, 3)

    p1 = p1.reshape(B, n_proposal1, 3, 3)
    p2 = p2.reshape(B, n_proposal1, 3, 3)
    dis = torch.norm((p1 - pred_ts) @ pred_rs - p2, dim=3).mean(2)
    idx = torch.topk(dis, n_proposal2, dim=1, largest=False)[1]
    pred_rs = torch.gather(pred_rs, 1, idx.reshape(B,n_proposal2,1,1).repeat(1,1,3,3))
    pred_ts = torch.gather(pred_ts, 1, idx.reshape(B,n_proposal2,1,1).repeat(1,1,1,3))

    # pose selection
    transformed_pts = (pts1.unsqueeze(1) - pred_ts) @ pred_rs
    transformed_pts = transformed_pts.reshape(B*n_proposal2, -1, 3)
    dis = torch.sqrt(pairwise_distance(transformed_pts, expand_model_pts))
    dis = dis.min(2)[0].reshape(B, n_proposal2, -1)
    scores = weights1.unsqueeze(1).sum(2) / ((dis * weights1.unsqueeze(1)).sum(2) + + 1e-8)
    # 5) 마스크 비교 게이트(ch1) — 기하 정렬 뒤·텍스처 검증 앞. 점수에 항을 더하지 않고
    #    '후보 자격'만 깎는다. 설정이 없으면 통째로 건너뛴다(= 예전 동작).
    mg = None if appe is None else appe.get('mask_gate')
    mp = None if appe is None else appe.get('map_prior')
    gate = None
    if mg is not None and mg.get('mask') is not None:   # 텐서가 실려 있을 때만
        keep1, mstat = mask_compare_gate(pred_rs, pred_ts.squeeze(2), model_pts, scores, mg)
        # ch2 — 합집합이지 교집합이 아니다. ch2 의 존재 이유가 ch1 이 거절할 가림
        # 프레임을 구제하는 것이라, 교집합이면 아무것도 구제되지 않는다.
        keep2 = (map_prior_gate(pred_rs, mp) if (mp is not None and mp.get('ref_R') is not None)
                 else torch.zeros_like(keep1))
        keep = keep1 | keep2
        keep.scatter_(1, scores.argmax(1, keepdim=True), True)     # 기하 1등은 언제나
        ch = keep1.int() | (keep2.int() << 1)                      # 1=mask 2=map 3=both
        gate = {'keep': keep, 'ch': ch, 'stat': mstat}
        mstat['n_map'] = keep2.sum(1)

    # appe=None 이면 아래 한 줄이 예전 그대로다(기본값). 켜면 텍스처로 상위 K 개를
    # 재정렬하고(verify 가 켜져 있으면) 뒤집힘 여부까지 판정해 info 에 담는다.
    idx = (scores.max(1)[1] if appe is None
           else appearance_rerank(pred_rs, pred_ts.squeeze(2), scores, appe, info, gate=gate))

    if gate is not None and info is not None:
        cases = info.pop('gate_cases', {})
        info['gate'] = _gate_rows(gate['stat'], idx, scores, info.pop('gate_aux', {}), mg)
        if bool(mg.get('dump_cases', True)):
            # 트리거: 승자가 바뀜 / 거른 후보가 있음 / fail-open 으로만 살아남음 /
            #        Stage V 가 확신하지 못함. 그 밖은 남길 이유가 없다.
            # dump_all 은 문턱 스윕용 — 중립 설정에서는 아무도 안 걸러서 트리거가
            # 한 번도 안 걸린다. 그때만 켠다(운영 기본은 트리거 방식).
            dump_all = bool(mg.get('dump_all', False))
            ver = info.get('verify') or [None] * len(info['gate'])
            for b, row in enumerate(info['gate']):
                v = (ver[b] or {}).get('verdict') if b < len(ver) else None
                trig = (dump_all
                        or row['changed'] or row['n_keep'] < row['n_eval']
                        or row['admit'] == 'rank1' or row['off']
                        or v in ('CORRECTED', 'AMBIGUOUS'))
                if trig and b in cases:
                    row['cases'] = cases[b]
    pred_R = torch.gather(pred_rs, 1, idx.reshape(B,1,1,1).repeat(1,1,3,3)).squeeze(1)
    pred_t = torch.gather(pred_ts, 1, idx.reshape(B,1,1,1).repeat(1,1,1,3)).squeeze(2).squeeze(1)
    return pred_R, pred_t



def compute_fine_Rt(
    atten,
    pts1,
    pts2,
    model_pts=None,
    dis_thres=0.15
):
    if model_pts is None:
        model_pts = pts2

    # compute pose
    WSVD = WeightedProcrustes(weight_thresh=0.0)
    assginment_mat = torch.softmax(atten, dim=2) * torch.softmax(atten, dim=1)
    label1 = torch.max(assginment_mat[:,1:,:], dim=2)[1]
    label2 = torch.max(assginment_mat[:,:,1:], dim=1)[1]

    assginment_mat = assginment_mat[:, 1:, 1:] * (label1>0).float().unsqueeze(2) * (label2>0).float().unsqueeze(1)
    # max_idx = torch.max(assginment_mat, dim=2, keepdim=True)[1]
    # pred_pts = torch.gather(pts2, 1, max_idx.expand_as(pts1))
    normalized_assginment_mat = assginment_mat / (assginment_mat.sum(2, keepdim=True) + 1e-6)
    pred_pts = normalized_assginment_mat @ pts2

    assginment_score = assginment_mat.sum(2)
    pred_R, pred_t = WSVD(pred_pts, pts1, assginment_score)

    # compute score
    pred_pts = (pts1 - pred_t.unsqueeze(1)) @ pred_R
    dis = torch.sqrt(pairwise_distance(pred_pts, model_pts)).min(2)[0]
    mask = (label1>0).float()
    pose_score = (dis < dis_thres).float()
    pose_score = (pose_score * mask).sum(1) / (mask.sum(1) + 1e-8)
    pose_score = pose_score * mask.mean(1)

    return pred_R, pred_t, pose_score



def weighted_procrustes(
    src_points,
    ref_points,
    weights=None,
    weight_thresh=0.0,
    eps=1e-5,
    return_transform=False,
    src_centroid = None,
    ref_centroid = None,
):
    r"""Compute rigid transformation from `src_points` to `ref_points` using weighted SVD.

    Modified from [PointDSC](https://github.com/XuyangBai/PointDSC/blob/master/models/common.py).

    Args:
        src_points: torch.Tensor (B, N, 3) or (N, 3)
        ref_points: torch.Tensor (B, N, 3) or (N, 3)
        weights: torch.Tensor (B, N) or (N,) (default: None)
        weight_thresh: float (default: 0.)
        eps: float (default: 1e-5)
        return_transform: bool (default: False)

    Returns:
        R: torch.Tensor (B, 3, 3) or (3, 3)
        t: torch.Tensor (B, 3) or (3,)
        transform: torch.Tensor (B, 4, 4) or (4, 4)
    """
    if src_points.ndim == 2:
        src_points = src_points.unsqueeze(0)
        ref_points = ref_points.unsqueeze(0)
        if weights is not None:
            weights = weights.unsqueeze(0)
        squeeze_first = True
    else:
        squeeze_first = False

    batch_size = src_points.shape[0]
    if weights is None:
        weights = torch.ones_like(src_points[:, :, 0])
    weights = torch.where(torch.lt(weights, weight_thresh), torch.zeros_like(weights), weights)
    weights = weights / (torch.sum(weights, dim=1, keepdim=True) + eps)
    weights = weights.unsqueeze(2)  # (B, N, 1)

    if src_centroid is None:
        src_centroid = torch.sum(src_points * weights, dim=1, keepdim=True)  # (B, 1, 3)
    elif len(src_centroid.size()) == 2:
        src_centroid = src_centroid.unsqueeze(1)
    src_points_centered = src_points - src_centroid  # (B, N, 3)

    if ref_centroid is None:
        ref_centroid = torch.sum(ref_points * weights, dim=1, keepdim=True)  # (B, 1, 3)
    elif len(ref_centroid.size()) == 2:
        ref_centroid = ref_centroid.unsqueeze(1)
    ref_points_centered = ref_points - ref_centroid  # (B, N, 3)

    H = src_points_centered.permute(0, 2, 1) @ (weights * ref_points_centered)
    U, _, V = torch.svd(H)
    Ut, V = U.transpose(1, 2), V
    eye = torch.eye(3).unsqueeze(0).repeat(batch_size, 1, 1).to(src_points.device)
    eye[:, -1, -1] = torch.sign(torch.det(V @ Ut))
    R = V @ eye @ Ut

    t = ref_centroid.permute(0, 2, 1) - R @ src_centroid.permute(0, 2, 1)
    t = t.squeeze(2)

    if return_transform:
        transform = torch.eye(4).unsqueeze(0).repeat(batch_size, 1, 1).to(DEVICE)
        transform[:, :3, :3] = R
        transform[:, :3, 3] = t
        if squeeze_first:
            transform = transform.squeeze(0)
        return transform
    else:
        if squeeze_first:
            R = R.squeeze(0)
            t = t.squeeze(0)
        return R, t


class WeightedProcrustes(nn.Module):
    def __init__(self, weight_thresh=0.5, eps=1e-5, return_transform=False):
        super(WeightedProcrustes, self).__init__()
        self.weight_thresh = weight_thresh
        self.eps = eps
        self.return_transform = return_transform

    def forward(self, src_points, tgt_points, weights=None,src_centroid = None,ref_centroid = None):
        return weighted_procrustes(
            src_points,
            tgt_points,
            weights=weights,
            weight_thresh=self.weight_thresh,
            eps=self.eps,
            return_transform=self.return_transform,
            src_centroid=src_centroid,
            ref_centroid=ref_centroid
        )

