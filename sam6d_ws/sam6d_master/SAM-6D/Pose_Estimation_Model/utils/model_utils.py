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


def _mask_rle(mask):
    """Compact row-major RLE used only by the static diagnostic report."""
    a = np.asarray(mask, dtype=np.uint8).reshape(-1)
    if not a.any():
        return []
    padded = np.pad(a, (1, 1))
    changes = np.flatnonzero(padded[1:] != padded[:-1])
    return [[int(start), int(end - start)] for start, end in changes.reshape(-1, 2)]


def _candidate_shape_metrics(pred_rs, pred_ts, model_pts, appe):
    """Project all 300 candidates and measure size/point-mask overlap independently.

    Size uses the bounding area of the accepted input points and projected CAD points in
    the same 224 crop. IoU uses a point-splat proxy, not a mesh silhouette.
    """
    mask = appe.get('shape_mask')
    K = appe.get('K')
    crop = appe.get('crop_bbox_yxyx')
    if mask is None or K is None or crop is None or model_pts is None:
        return None
    mask = mask.bool()
    if mask.ndim != 3 or not bool(mask.flatten(1).any(1).all()):
        return None
    B, H, W = mask.shape
    P = pred_rs.size(1)
    ver = appe.get('verify') or {}
    chunk = max(1, int(ver.get('projection_chunk', 32)))
    base_splat = max(0, int(ver.get('point_splat_radius_px', 2)))
    adaptive_splat = bool(ver.get('adaptive_point_splat', True))
    closing = max(0, int(ver.get('closing_radius_px', 1)))

    obs_area = mask.flatten(1).sum(1).float()
    obs_bbox_area, obs_bbox = [], []
    for b in range(B):
        yy, xx = torch.where(mask[b])
        x0, x1 = xx.min(), xx.max()
        y0, y1 = yy.min(), yy.max()
        obs_bbox_area.append((x1 - x0 + 1).float() * (y1 - y0 + 1).float())
        obs_bbox.append([int(x0), int(y0), int(x1), int(y1)])
    obs_bbox_area = torch.stack(obs_bbox_area)

    values = {key: [] for key in ('rendered_bbox_area_px', 'size_ratio', 'mask_iou',
                                   'coverage', 'rendered_mask_area_px', 'projection_valid')}
    for start in range(0, P, chunk):
        stop = min(P, start + chunk)
        R = pred_rs[:, start:stop]
        t = pred_ts[:, start:stop, None, :]
        cam = torch.einsum('bmj,bcij->bcmi', model_pts, R) + t
        z = cam[..., 2]
        z_ok = torch.isfinite(z) & (z > 1e-6)
        u = K[:, None, None, 0, 0] * cam[..., 0] / z.clamp_min(1e-6) + K[:, None, None, 0, 2]
        v = K[:, None, None, 1, 1] * cam[..., 1] / z.clamp_min(1e-6) + K[:, None, None, 1, 2]
        y1c, y2c, x1c, x2c = [crop[:, i, None, None] for i in range(4)]
        uc = (u - x1c) * float(W) / (x2c - x1c).clamp_min(1.0)
        vc = (v - y1c) * float(H) / (y2c - y1c).clamp_min(1.0)
        finite = z_ok & torch.isfinite(uc) & torch.isfinite(vc)
        inside = (finite & (uc >= 0) & (uc < W) & (vc >= 0) & (vc < H))
        inf = torch.full_like(uc, float('inf'))
        ninf = torch.full_like(uc, float('-inf'))
        umin = torch.where(inside, uc, inf).min(2)[0]
        umax = torch.where(inside, uc, ninf).max(2)[0]
        vmin = torch.where(inside, vc, inf).min(2)[0]
        vmax = torch.where(inside, vc, ninf).max(2)[0]
        rotation_finite = torch.isfinite(R).flatten(2).all(2)
        translation_finite = torch.isfinite(t).flatten(2).all(2)
        orthogonal = torch.linalg.norm(
            R.transpose(-1, -2) @ R - torch.eye(3, device=R.device, dtype=R.dtype),
            dim=(-2, -1)) <= 1e-2
        proper = torch.linalg.det(R) > 0.99
        pose_valid = rotation_finite & translation_finite & orthogonal & proper
        projection_valid = inside.any(2) & pose_valid
        rendered_bbox_area = ((umax - umin + 1).clamp_min(0) *
                              (vmax - vmin + 1).clamp_min(0))
        rendered_bbox_area = torch.where(
            projection_valid, rendered_bbox_area, torch.zeros_like(rendered_bbox_area))

        ui = uc.round().long().clamp(0, W - 1)
        vi = vc.round().long().clamp(0, H - 1)
        flat = torch.zeros(B * (stop - start), H * W, device=cam.device)
        flat.scatter_add_(1, (vi * W + ui).reshape(B * (stop - start), -1),
                          inside.reshape(B * (stop - start), -1).float())
        projected = (flat > 0).reshape(B * (stop - start), 1, H, W).float()
        # A fixed radius is the lower bound. Sparse/far projections receive one or two
        # extra pixels so the 8192 surface samples approximate a closed silhouette.
        # Apply that radius per candidate: using the maximum radius of a whole chunk
        # makes a candidate's IoU depend on unrelated candidates and chunk ordering.
        if adaptive_splat:
            valid_counts = inside.sum(2).float().clamp_min(1.0)
            bbox_pixels = rendered_bbox_area.clamp_min(1.0)
            spacing = torch.sqrt(bbox_pixels / valid_counts)
            max_splat = max(4, base_splat)
            splat_radii = torch.ceil(spacing).long().clamp(
                min=base_splat, max=max_splat)
        else:
            max_splat = base_splat
            splat_radii = torch.full_like(rendered_bbox_area, base_splat, dtype=torch.long)
        splat_radii = splat_radii.reshape(-1)
        expanded = torch.zeros_like(projected)
        for splat in range(max_splat + 1):
            selected = splat_radii == splat
            if not bool(selected.any().item()):
                continue
            if splat:
                expanded[selected] = F.max_pool2d(
                    projected[selected], 2 * splat + 1, stride=1, padding=splat)
            else:
                expanded[selected] = projected[selected]
        projected = expanded
        if closing:
            kernel = 2 * closing + 1
            projected = F.max_pool2d(projected, kernel, stride=1, padding=closing)
            projected = -F.max_pool2d(-projected, kernel, stride=1, padding=closing)
        projected = projected.bool().reshape(B, stop - start, H, W)
        observed = mask[:, None]
        intersection = (projected & observed).flatten(2).sum(2).float()
        rendered_area = projected.flatten(2).sum(2).float()
        union = rendered_area + obs_area[:, None] - intersection

        values['rendered_bbox_area_px'].append(rendered_bbox_area)
        values['size_ratio'].append(obs_bbox_area[:, None] / rendered_bbox_area.clamp_min(1.0))
        values['mask_iou'].append(intersection / union.clamp_min(1.0))
        values['coverage'].append(intersection / obs_area[:, None].clamp_min(1.0))
        values['rendered_mask_area_px'].append(rendered_area)
        values['projection_valid'].append(projection_valid)

    out = {key: torch.cat(parts, 1) for key, parts in values.items()}
    out.update({'observed_mask_area_px': obs_area, 'observed_bbox_area_px': obs_bbox_area,
                'observed_bbox_224': obs_bbox,
                'observed_mask_rle': [_mask_rle(mask[b].detach().cpu().numpy()) for b in range(B)],
                'mask_size': [H, W], 'point_splat_radius_px': base_splat,
                'adaptive_point_splat': adaptive_splat,
                'closing_radius_px': closing})
    return out


def sequential_candidate_select(pred_rs, pred_ts_m, geo_scores, texture_scores,
                                shape, names, ver, proposal_ids=None):
    """Apply Mask -> Texture -> pose-convergence selection to all geometry candidates.

    The return index always identifies an existing proposal. Rejected batches receive the
    geometry winner as an inert placeholder so batched fine inference stays shape-stable;
    ``accepted`` is authoritative and prevents that placeholder from being published.
    """
    B, P = geo_scores.shape
    mask_min = float(ver.get('mask_iou_min', ver.get('iou_min', 0.420998)))
    texture_min = float(ver.get('texture_min_score', 0.449562))
    rot_limit = float(ver.get('cluster_rotation_deg', 20.0))
    trans_limit_m = float(ver.get('cluster_translation_mm', 25.0)) / 1000.0
    occupancy_min = float(ver.get('cluster_min_occupancy', 0.5))
    sym_tab = ver.get('symmetry_axes') or {}
    sym_step = int(ver.get('sym_step_deg', 10))
    selected_out, rows = [], []

    for b in range(B):
        geo_order = torch.argsort(geo_scores[b], descending=True, stable=True)
        rank_geo = torch.empty(P, dtype=torch.long, device=geo_scores.device)
        rank_geo[geo_order] = torch.arange(P, device=geo_scores.device)
        fallback = int(geo_order[0].item())
        R = pred_rs[b]
        t = pred_ts_m[b]
        pose_valid = (torch.isfinite(R).flatten(1).all(1)
                      & torch.isfinite(t).flatten(1).all(1)
                      & torch.isfinite(geo_scores[b]))
        pose_valid &= (torch.linalg.norm(
            R.transpose(-1, -2) @ R - torch.eye(3, device=R.device, dtype=R.dtype),
            dim=(-2, -1)) <= 1e-2) & (torch.linalg.det(R) > 0.99)
        if shape is None:
            projection_valid = torch.zeros(P, dtype=torch.bool, device=R.device)
            mask_iou = torch.full((P,), float('nan'), device=R.device)
        else:
            projection_valid = shape['projection_valid'][b].bool()
            mask_iou = shape['mask_iou'][b]
        mask_pass = pose_valid & projection_valid & torch.isfinite(mask_iou) & (mask_iou >= mask_min)
        texture_pass = (mask_pass & torch.isfinite(texture_scores[b])
                        & (texture_scores[b] >= texture_min))
        survivors = torch.where(texture_pass)[0]
        accepted = False
        reason = None
        occupancy = 0.0
        cluster = torch.empty(0, dtype=torch.long, device=R.device)
        center = None

        if not bool((pose_valid & projection_valid).any().item()):
            reason = 'candidate_projection_invalid'
        elif not bool(mask_pass.any().item()):
            reason = 'mask_filter_empty'
        elif not bool(texture_pass.any().item()):
            reason = 'texture_filter_empty'
        elif survivors.numel() == 1:
            cluster = survivors
            center = int(survivors[0].item())
            occupancy = 1.0
            accepted = True
        else:
            sym = _sym_group(sym_tab.get(names[b]), sym_step, R.device, R.dtype)
            angles = _pairwise_angle_deg(R, sym)
            translations = torch.cdist(t, t)
            best_key = None
            best_cluster = None
            best_center = None
            for candidate in survivors.tolist():
                members = survivors[(angles[candidate, survivors] <= rot_limit)
                                    & (translations[candidate, survivors] <= trans_limit_m)]
                # Largest neighbourhood first; ties use the better geometry-ranked centre.
                key = (int(members.numel()), -int(rank_geo[candidate].item()))
                if best_key is None or key > best_key:
                    best_key, best_cluster, best_center = key, members, candidate
            cluster, center = best_cluster, best_center
            occupancy = float(cluster.numel()) / float(survivors.numel())
            accepted = occupancy >= occupancy_min
            if not accepted:
                reason = 'convergence_insufficient'

        chosen = fallback
        if accepted:
            chosen = int(cluster[torch.argmin(rank_geo[cluster])].item())
        selected_out.append(chosen)
        proposal = (torch.arange(P, device=R.device) if proposal_ids is None
                    else proposal_ids[b])
        candidates = []
        for original in geo_order.tolist():
            candidate_row = {
                'rank_geo': int(rank_geo[original].item()),
                'index300': int(original),
                'proposal6000_index': int(proposal[original].item()),
                'geometry_score': _rounded_finite(geo_scores[b, original].item(), 8),
                'mask_iou': (_rounded_finite(mask_iou[original].item(), 6)
                             if torch.isfinite(mask_iou[original]) else None),
                'texture_score': (_rounded_finite(texture_scores[b, original].item(), 8)
                                  if torch.isfinite(texture_scores[b, original]) else None),
                'pose_valid': bool(pose_valid[original].item()),
                'projection_valid': bool(projection_valid[original].item()),
                'mask_pass': bool(mask_pass[original].item()),
                'texture_pass': bool(texture_pass[original].item()),
                'cluster_member': bool((cluster == original).any().item()),
            }
            if shape is not None and 'coverage' in shape:
                candidate_row.update({
                    'coverage': _rounded_finite(shape['coverage'][b, original].item(), 6),
                    'size_ratio': _rounded_finite(
                        shape['size_ratio'][b, original].item(), 6),
                    'rendered_mask_area_px': int(
                        shape['rendered_mask_area_px'][b, original].item()),
                })
            candidates.append(candidate_row)
        selected_candidate = candidates[int(rank_geo[chosen].item())]
        rows.append({
            'selection_method': 'geometry_mask_texture_convergence',
            'accepted': accepted,
            'rejection_reason': reason,
            'rank_geo': int(rank_geo[chosen].item()),
            'selected_index300': chosen if accepted else None,
            'selected_proposal6000_index': (int(proposal[chosen].item()) if accepted else None),
            'mask_iou_min': mask_min,
            'texture_score_min': texture_min,
            'mask_survivors': int(mask_pass.sum().item()),
            'texture_survivors': int(texture_pass.sum().item()),
            'cluster_size': int(cluster.numel()),
            'cluster_occupancy': round(occupancy, 6),
            'cluster_center_rank_geo': (None if center is None else
                                        int(rank_geo[center].item())),
            'mask_iou': selected_candidate.get('mask_iou'),
            'texture_score': selected_candidate.get('texture_score'),
            'coverage': selected_candidate.get('coverage'),
            'size_ratio': selected_candidate.get('size_ratio'),
            'candidates': candidates,
        })
    return torch.as_tensor(selected_out, dtype=torch.long, device=geo_scores.device), rows


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


def _reference_stage_summary(pred_rs, pred_ts, appe, stage, radius):
    """GT가 주어진 진단 실행에서 후보 단계의 포함 여부만 작게 보존한다.

    전체 6,000 행렬을 파일로 내보내지 않고 실제 생성 지점에서 count/minimum을 계산한다.
    GT가 없으면 X가 아니라 ``gt_missing``을 기록한다.
    """
    diag = appe.get('diagnostic') or {}
    out = []
    valid = appe.get('diagnostic_ref_valid')
    ref_r = appe.get('diagnostic_ref_R')
    ref_t = appe.get('diagnostic_ref_t')
    rot_limit = float(diag.get('rotation_threshold_deg', 30.0))
    trans_limit = float(diag.get('translation_threshold_mm', 100.0)) / 1000.0
    names = appe.get('names') or [None] * pred_rs.size(0)
    for b in range(pred_rs.size(0)):
        base = {'stage': stage, 'total': int(pred_rs.size(1)),
                'rotation_threshold_deg': rot_limit,
                'translation_threshold_mm': trans_limit * 1000.0}
        if valid is None or ref_r is None or ref_t is None or not bool(valid[b].item()):
            out.append({**base, 'status': 'gt_missing', 'present': None})
            continue
        R = pred_rs[b]
        target = ref_r[b]
        sym = _sym_group((diag.get('symmetry_axes') or {}).get(names[b]),
                         int(diag.get('sym_step_deg', 10)), R.device, R.dtype)
        # trace(R_candidate^T @ (R_gt @ S)); [P,G]
        equivalent = torch.einsum('ij,gjk->gik', target, sym)
        trace = torch.einsum('pij,gij->pg', R, equivalent).max(1)[0]
        angle = torch.rad2deg(torch.arccos(((trace - 1.0) * 0.5).clamp(-1.0, 1.0)))
        scale = 1.0 if radius is None else radius[b]
        trans = torch.linalg.norm(pred_ts[b] * scale - ref_t[b].reshape(1, 3), dim=1)
        pose_ok = (angle <= rot_limit) & (trans <= trans_limit)
        out.append({**base, 'status': 'available', 'present': bool(pose_ok.any().item()),
                    'match_count': int(pose_ok.sum().item()),
                    'rotation_match_count': int((angle <= rot_limit).sum().item()),
                    'min_rotation_error_deg': round(float(angle.min().item()), 4),
                    'min_translation_error_mm': round(float(trans.min().item() * 1000.0), 3)})
    return out


_RESIDUAL_HISTOGRAM_EDGES = (0.0, 0.0005, 0.001, 0.002, 0.005, 0.01,
                             0.02, 0.05, 0.1, 0.2, 0.5, 1.0)


def _candidate_reference_errors(pred_rs, pred_ts, appe, radius):
    """Return symmetry-aware candidate errors without influencing selection."""
    diag = appe.get('diagnostic') or {}
    valid = appe.get('diagnostic_ref_valid')
    ref_r = appe.get('diagnostic_ref_R')
    ref_t = appe.get('diagnostic_ref_t')
    names = appe.get('names') or [None] * pred_rs.size(0)
    rot_limit = float(diag.get('rotation_threshold_deg', 30.0))
    trans_limit = float(diag.get('translation_threshold_mm', 100.0)) / 1000.0
    rows = []
    for b in range(pred_rs.size(0)):
        if valid is None or ref_r is None or ref_t is None or not bool(valid[b].item()):
            rows.append(None)
            continue
        R = pred_rs[b]
        sym = _sym_group((diag.get('symmetry_axes') or {}).get(names[b]),
                         int(diag.get('sym_step_deg', 10)), R.device, R.dtype)
        equivalent = torch.einsum('ij,gjk->gik', ref_r[b], sym)
        trace = torch.einsum('pij,gij->pg', R, equivalent).max(1)[0]
        angle = torch.rad2deg(torch.arccos(((trace - 1.0) * 0.5).clamp(-1.0, 1.0)))
        scale = 1.0 if radius is None else radius[b]
        translation = torch.linalg.norm(
            pred_ts[b] * scale - ref_t[b].reshape(1, 3), dim=1)
        pose_valid = (torch.isfinite(R).flatten(1).all(1)
                      & torch.isfinite(pred_ts[b]).flatten(1).all(1)
                      & torch.isfinite(angle) & torch.isfinite(translation))
        rows.append({
            'rotation_error_deg': angle,
            'translation_error_mm': translation * 1000.0,
            'pose_valid': pose_valid,
            'correct': pose_valid & (angle <= rot_limit) & (translation <= trans_limit),
        })
    return rows


def _correctness_metadata(diag, name):
    axis = (diag.get('symmetry_axes') or {}).get(name)
    return {
        'rotation_threshold_deg': float(diag.get('rotation_threshold_deg', 30.0)),
        'translation_threshold_mm': float(diag.get('translation_threshold_mm', 100.0)),
        'symmetry_axis': None if axis is None else [float(value) for value in axis],
        'symmetry_step_deg': int(diag.get('sym_step_deg', 10)),
    }


def _rounded_finite(value, digits):
    number = float(value)
    return round(number, digits) if math.isfinite(number) else None


def _compact_distribution(values):
    """Fixed-bin aggregate used for 6,000-candidate residuals."""
    values = values.detach().float()
    finite = values[torch.isfinite(values)]
    edges = values.new_tensor(_RESIDUAL_HISTOGRAM_EDGES)
    if not finite.numel():
        return {'count': 0, 'missing_count': int(values.numel()),
                'edges': list(_RESIDUAL_HISTOGRAM_EDGES),
                'bins': [0] * (len(_RESIDUAL_HISTOGRAM_EDGES) - 1),
                'overflow': 0, 'quantiles': {}}
    # bucketize produces 1..len(edges) for finite values. Values below zero are
    # invalid residuals and are counted as missing rather than mixed into bin 0.
    nonnegative = finite[finite >= 0]
    bucket = torch.bucketize(nonnegative, edges, right=True) - 1
    bins = [int((bucket == i).sum().item()) for i in range(len(edges) - 1)]
    quantile_levels = values.new_tensor([0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0])
    q = torch.quantile(nonnegative, quantile_levels) if nonnegative.numel() else None
    return {
        'count': int(nonnegative.numel()),
        'missing_count': int(values.numel() - nonnegative.numel()),
        'edges': list(_RESIDUAL_HISTOGRAM_EDGES),
        'bins': bins,
        'overflow': int((bucket >= len(edges) - 1).sum().item()),
        'quantiles': ({} if q is None else {
            key: round(float(value), 8)
            for key, value in zip(('min', 'p05', 'p25', 'p50', 'p75', 'p95', 'max'), q)
        }),
    }


def _stage6000_score_analysis(pred_rs, pred_ts, residual, appe, radius):
    """Aggregate initial 3-point residuals; never serialize 6,000 raw rows."""
    errors = _candidate_reference_errors(pred_rs, pred_ts, appe, radius)
    diag = appe.get('diagnostic') or {}
    names = appe.get('names') or [None] * pred_rs.size(0)
    rows = []
    for b in range(pred_rs.size(0)):
        base = {
            'score_name': 'initial_3point_residual',
            'score_semantics': 'lower_is_better; not a PEM geometry score',
            'units': 'normalized_object_radius',
            'candidate_count': int(residual.size(1)),
            'correctness': _correctness_metadata(diag, names[b]),
            'overall': _compact_distribution(residual[b]),
        }
        if errors[b] is None:
            rows.append({**base, 'gt_status': 'unavailable',
                         'correct': None, 'incorrect': None})
            continue
        valid = errors[b]['pose_valid']
        correct = errors[b]['correct']
        rows.append({**base, 'gt_status': 'available',
                     'correct': _compact_distribution(residual[b][correct]),
                     'incorrect': _compact_distribution(residual[b][valid & ~correct]),
                     'invalid_pose': _compact_distribution(residual[b][~valid])})
    return rows


def _pointwise_evidence(pm, fm, cm, po, fo, co, nn, texture_distance, rgb_choose,
                        geo_distance, geo_valid, geo_uv, radius, indices,
                        rank_g, rank_texture, sample_stride):
    """상위 소수 후보의 실제 최근접 대응/특징 점수를 JSON 친화형으로 축약한다."""
    rows = []
    step = max(1, int(sample_stride))
    uv = None if rgb_choose is None else rgb_choose.detach().cpu().numpy()
    for k in indices:
        ids = torch.arange(0, pm.size(0), step, device=pm.device)
        chosen_nn = nn[k][ids]
        texture_dist_mm = texture_distance[k][ids] * radius * 1000.0
        feat = (fm[ids] * fo[chosen_nn]).sum(1)
        col = None
        if cm is not None and co is not None:
            col = (cm[ids] * co[chosen_nn]).mean(1)
        td = texture_dist_mm.detach().cpu().numpy()
        f = feat.detach().cpu().numpy()
        texture_points = []
        for q, src in enumerate(ids.detach().cpu().numpy().tolist()):
            px = None
            if uv is not None:
                flat = int(uv[src])
                px = [flat % 224, flat // 224]
            texture_points.append({
                'uv_224': px,
                'template_distance_mm': round(float(td[q]), 3),
                'feature_cosine': round(float(f[q]), 4),
                'color_similarity': (None if col is None else
                                     round(float(col[q].item()), 4)),
            })
        exact = geo_distance[k]
        valid_mask = (geo_valid > 0) if geo_valid is not None else None
        exact = exact[valid_mask] if valid_mask is not None else exact
        gd = (exact * radius * 1000.0).detach().cpu().numpy()
        selected_uv = (geo_uv[valid_mask] if geo_uv is not None and valid_mask is not None
                       else geo_uv)
        guv = None if selected_uv is None else selected_uv.detach().cpu().numpy()
        finite = np.isfinite(gd)
        gd = gd[finite]
        if guv is not None:
            guv = guv[finite]
        geometry_points = []
        for q, value in enumerate(gd.tolist()):
            px = None
            if guv is not None:
                flat = int(guv[q])
                px = [flat % 224, flat // 224]
            geometry_points.append({'uv_224': px, 'distance_mm': round(float(value), 3)})
        threshold = float(max(2.0, min(20.0, radius * 1000.0 * 0.05)))
        geometry_summary = {
            'point_count': len(geometry_points), 'threshold_mm': round(threshold, 3),
            'overlap_count': int((gd <= threshold).sum()) if len(gd) else 0,
            'overlap_ratio': round(float(np.mean(gd <= threshold)), 4) if len(gd) else None,
            'mean_distance_mm': round(float(np.mean(gd)), 3) if len(gd) else None,
            'p90_distance_mm': round(float(np.percentile(gd, 90)), 3) if len(gd) else None,
        }
        rows.append({
            'rank_geo': int(rank_g[int(k)]),
            'rank_texture': int(rank_texture[int(k)]),
            'geometry': geometry_summary,
            'texture': {'feature_mean': round(float(np.mean(f)), 4),
                        'color_mean': (None if col is None else
                                       round(float(col.mean().item()), 4))},
            'geometry_points': geometry_points,
            'texture_points': texture_points,
        })
    return rows


def _attach_explorer_v2(info, pred_rs, physical_t, geo_scores, texture_scores,
                        shape, appe, decisions, selected):
    """Expose lossless arrays only to the compact Explorer recorder."""
    enabled = ((appe.get('diagnostic') or {}).get('explorer_v2') or {}).get('enabled')
    if info is None or not enabled:
        return
    B, P = geo_scores.shape
    proposal_ids = appe.get('_proposal_ids')
    payloads = []
    for b in range(B):
        rank_geo = torch.empty(P, dtype=torch.long, device=geo_scores.device)
        rank_geo[torch.argsort(geo_scores[b], descending=True, stable=True)] = torch.arange(
            P, device=geo_scores.device)
        rank_texture = torch.empty(P, dtype=torch.long, device=geo_scores.device)
        rank_texture[torch.argsort(torch.nan_to_num(texture_scores[b], nan=float('-inf')),
                                   descending=True, stable=True)] = torch.arange(
            P, device=geo_scores.device)
        R, t = pred_rs[b], physical_t[b]
        flags = torch.zeros(P, dtype=torch.int32, device=geo_scores.device)
        pose_valid = (torch.isfinite(R).flatten(1).all(1) & torch.isfinite(t).all(1)
                      & torch.isfinite(geo_scores[b]))
        pose_valid &= (torch.linalg.norm(
            R.transpose(-1, -2) @ R - torch.eye(3, device=R.device, dtype=R.dtype),
            dim=(-2, -1)) <= 1e-2) & (torch.linalg.det(R) > 0.99)
        flags |= pose_valid.int()
        # Bit 6 records whether Texture was actually measured.  It reuses the
        # existing uint16 flags field, so the compact candidate record stays 80 B.
        flags |= torch.isfinite(texture_scores[b]).int() * 64
        if shape is not None:
            flags |= shape['projection_valid'][b].int() * 2
        decision = decisions[b] if decisions else None
        if decision:
            for row in decision.get('candidates', []):
                original = int(row['index300'])
                flags[original] |= (4 if row.get('mask_pass') else 0)
                flags[original] |= (8 if row.get('texture_pass') else 0)
                flags[original] |= (16 if row.get('cluster_member') else 0)
            if decision.get('accepted') and decision.get('selected_index300') is not None:
                flags[int(decision['selected_index300'])] |= 32
        else:
            flags[int(selected[b].item())] |= 32
        nan = torch.full((P,), float('nan'), device=geo_scores.device)
        payloads.append({
            'R': R.detach().cpu().numpy(), 't_m': t.detach().cpu().numpy(),
            'geometry': geo_scores[b].detach().cpu().numpy(),
            'mask_iou': (nan if shape is None else shape['mask_iou'][b]).detach().cpu().numpy(),
            'texture': texture_scores[b].detach().cpu().numpy(),
            'coverage': (nan if shape is None else shape['coverage'][b]).detach().cpu().numpy(),
            'size_ratio': (nan if shape is None else shape['size_ratio'][b]).detach().cpu().numpy(),
            'proposal6000': ((torch.arange(P, device=R.device) if proposal_ids is None
                              else proposal_ids[b]).detach().cpu().numpy()),
            'index300': np.arange(P, dtype=np.uint16),
            'rank_geo': rank_geo.detach().cpu().numpy(),
            'rank_texture': rank_texture.detach().cpu().numpy(),
            'flags': flags.detach().cpu().numpy(),
            'replay': {
                'source_pixel_index': appe['source_pixel_index'][b].detach().cpu().numpy(),
                'coarse_fps_index': appe['coarse_fps_idx'][b].detach().cpu().numpy(),
                'coarse_valid': appe['_geo_point_valid'][b].detach().cpu().numpy(),
                'cad_sample_index': appe['cad_sample_index'][b].detach().cpu().numpy(),
            },
            'crop_bbox_yxyx': (appe['crop_bbox_yxyx'][b].detach().cpu().numpy()
                               if appe.get('crop_bbox_yxyx') is not None else []),
            'decision': decision or {'accepted': True,
                                     'selected_index300': int(selected[b].item())},
        })
    info['pem_explorer'] = payloads


def independent_candidate_verify(pred_rs, pred_ts, geo_scores, appe, info=None):
    """Measure all candidates and, when enabled, apply the production filter chain."""
    B, P = geo_scores.shape
    topk = int(min(appe.get('topk', 100), P))
    stride = max(1, int(appe.get('stride', 1)))
    ver = appe.get('verify') or {}
    diag = appe.get('diagnostic') or {}
    analysis_cfg = diag.get('score_analysis') or {}
    analysis_on = bool(diag.get('enabled') and analysis_cfg.get('enabled'))
    explorer_cfg = diag.get('explorer_v2') or {}
    explorer_on = bool(diag.get('enabled') and explorer_cfg.get('enabled'))
    capture_profile = explorer_cfg.get(
        'capture_profile', 'exhaustive_visualization')
    if explorer_on and capture_profile not in (
            'exhaustive_visualization', 'realtime_inference'):
        raise ValueError(
            'diagnostic.explorer_v2.capture_profile must be '
            "'exhaustive_visualization' or 'realtime_inference'")
    exhaustive_capture = explorer_on and capture_profile == 'exhaustive_visualization'
    realtime_capture = explorer_on and capture_profile == 'realtime_inference'
    production_on = bool(ver.get('enabled', False))
    # Realtime capture is a measurement ceiling: even if a score-analysis block
    # is present, it must not widen the production texture population.
    measured_count = P if (production_on or exhaustive_capture or
                           (analysis_on and not realtime_capture)) else topk
    dump = max(int(ver.get('dump_topk', 0)), int(diag.get('topn', 0)))
    diag_topn = max(0, int(diag.get('topn', 0))) if diag.get('enabled') else 0
    diag_stride = max(1, int(diag.get('evidence_stride', 8)))
    radius = appe.get('radius')
    have_col = ('dense_cm' in appe) and ('dense_co' in appe)
    shape = _candidate_shape_metrics(
        pred_rs, pred_ts, appe.get('_projection_model_pts'), appe)
    if production_on:
        texture_by_original = torch.full_like(geo_scores, float('nan'))
        mask_min = float(ver.get('mask_iou_min', ver.get('iou_min', 0.420998)))
        texture_chunk = max(1, int(ver.get('texture_chunk', 16)))
        for b in range(B):
            if shape is None:
                continue
            R_all, t_all = pred_rs[b], pred_ts[b]
            pose_valid = (torch.isfinite(R_all).flatten(1).all(1)
                          & torch.isfinite(t_all).flatten(1).all(1)
                          & torch.isfinite(geo_scores[b])
                          & (torch.linalg.det(R_all) > 0.99))
            mask_pass = (pose_valid & shape['projection_valid'][b]
                         & torch.isfinite(shape['mask_iou'][b])
                         & (shape['mask_iou'][b] >= mask_min))
            # Keep the production candidate set and chunk boundaries byte-for-byte
            # identical. Explorer-only candidates are measured in a second pass after
            # the authoritative selection below.
            candidates = torch.where(mask_pass)[0]
            if not candidates.numel():
                continue
            pm = appe['dense_pm'][b][::stride]
            fm = F.normalize(appe['dense_fm'][b][::stride], dim=1)
            po = appe['dense_po'][b]
            fo = F.normalize(appe['dense_fo'][b], dim=1)
            for subset in candidates.split(texture_chunk):
                R = R_all[subset]
                t = t_all[subset].reshape(-1, 1, 3)
                obj = torch.einsum('kji,knj->kni', R, pm.unsqueeze(0) - t)
                nearest = torch.cdist(
                    obj, po.unsqueeze(0).expand(obj.size(0), -1, -1)).argmin(2)
                texture_by_original[b, subset] = torch.einsum(
                    'nd,knd->kn', fm, fo[nearest]).mean(1)
        physical_t = pred_ts.clone()
        if radius is not None:
            physical_t = physical_t * radius.reshape(-1, 1, 1)
        selected, rows = sequential_candidate_select(
            pred_rs, physical_t, geo_scores, texture_by_original, shape,
            appe.get('names') or [None] * B, ver, appe.get('_proposal_ids'))
        if exhaustive_capture:
            texture_chunk = max(1, int(ver.get('texture_chunk', 16)))
            for b in range(B):
                R_all, t_all = pred_rs[b], pred_ts[b]
                pose_valid = (torch.isfinite(R_all).flatten(1).all(1)
                              & torch.isfinite(t_all).flatten(1).all(1)
                              & torch.isfinite(geo_scores[b])
                              & (torch.linalg.det(R_all) > 0.99))
                missing = torch.where(pose_valid & ~torch.isfinite(texture_by_original[b]))[0]
                if not missing.numel():
                    continue
                pm = appe['dense_pm'][b][::stride]
                fm = F.normalize(appe['dense_fm'][b][::stride], dim=1)
                po = appe['dense_po'][b]
                fo = F.normalize(appe['dense_fo'][b], dim=1)
                for subset in missing.split(texture_chunk):
                    obj = torch.einsum(
                        'kji,knj->kni', R_all[subset],
                        pm.unsqueeze(0) - t_all[subset].reshape(-1, 1, 3))
                    nearest = torch.cdist(
                        obj, po.unsqueeze(0).expand(obj.size(0), -1, -1)).argmin(2)
                    texture_by_original[b, subset] = torch.einsum(
                        'nd,knd->kn', fm, fo[nearest]).mean(1)
        _attach_explorer_v2(info, pred_rs, physical_t, geo_scores,
                            texture_by_original, shape, appe, rows, selected)
        for b, decision in enumerate(rows):
            if shape is not None:
                decision['shape_mask'] = {
                    'size': shape['mask_size'],
                    'rle': shape['observed_mask_rle'][b],
                    'bbox_224': shape['observed_bbox_224'][b],
                    'crop_bbox_yxyx': [round(float(v), 3) for v in
                                       appe['crop_bbox_yxyx'][b].detach().cpu().tolist()],
                    'point_splat_radius_px': shape['point_splat_radius_px'],
                    'closing_radius_px': shape['closing_radius_px'],
                    'provenance': 'original ISM mask; 8192-point adaptive-splat proxy',
                }
        if info is not None:
            info['verify'] = rows
            if analysis_on:
                errors = _candidate_reference_errors(pred_rs, pred_ts, appe, radius)
                stages = []
                for b, decision in enumerate(rows):
                    finite_geo = geo_scores[b][torch.isfinite(geo_scores[b])]
                    low = float(finite_geo.min().item()) if finite_geo.numel() else math.nan
                    high = float(finite_geo.max().item()) if finite_geo.numel() else math.nan
                    span = high - low
                    for candidate in decision['candidates']:
                        original = candidate['index300']
                        geometry = candidate['geometry_score']
                        candidate.update({
                            'R': [[round(float(value), 6) for value in matrix_row]
                                  for matrix_row in pred_rs[b, original].detach().cpu().numpy()],
                            't_mm': [round(float(value) * 1000.0, 3) for value in
                                     physical_t[b, original].detach().cpu().numpy()],
                            'geometry_score_detection_normalized': (
                                None if geometry is None or not math.isfinite(span) or span <= 0
                                else _rounded_finite((geometry - low) / span, 8)),
                            'geometry_selected': bool(
                                decision['accepted'] and original == decision['selected_index300']),
                            'missing_reason': (None if candidate['projection_valid'] else
                                               'invalid_projection'),
                        })
                        error = errors[b]
                        candidate_valid = (candidate['pose_valid'] and
                                           (error is None or bool(
                                               error['pose_valid'][original].item())))
                        candidate['candidate_valid'] = candidate_valid
                        if error is None:
                            candidate.update({'gt_status': 'unavailable', 'correct': None,
                                              'rotation_error_deg': None,
                                              'translation_error_mm': None})
                        elif not candidate_valid:
                            candidate.update({'gt_status': 'available', 'correct': None,
                                              'rotation_error_deg': None,
                                              'translation_error_mm': None,
                                              'missing_reason': 'invalid_candidate_pose'})
                        else:
                            candidate.update({
                                'gt_status': 'available',
                                'correct': bool(error['correct'][original].item()),
                                'rotation_error_deg': _rounded_finite(
                                    error['rotation_error_deg'][original].item(), 5),
                                'translation_error_mm': _rounded_finite(
                                    error['translation_error_mm'][original].item(), 5),
                            })
                    stages.append({
                        'schema_version': 1,
                        'selection_method': decision['selection_method'],
                        'candidate_count': P,
                        'selected_index300': decision['selected_index300'],
                        'correctness': _correctness_metadata(
                            diag, (appe.get('names') or [None] * B)[b]),
                        'channels': {
                            'geometry_score': {'direction': 'higher_is_better',
                                               'kind': 'PEM geometry'},
                            'geometry_score_detection_normalized': {
                                'direction': 'higher_is_better',
                                'normalization': 'per-detection min-max'},
                            'texture_score': {'direction': 'higher_is_better',
                                              'kind': 'sequential_after_mask'},
                            'mask_iou': {'direction': 'higher_is_better',
                                         'kind': '8192-point silhouette proxy'},
                        },
                        'candidates': decision['candidates'],
                    })
                existing = info.get('score_analysis') or [{} for _ in range(B)]
                info['score_analysis'] = [
                    {**existing[b], 'stage300': stages[b]} for b in range(B)]
            if not analysis_on:
                requested = max(0, int(ver.get('dump_topk', 0)))
                for decision in rows:
                    if requested:
                        decision['candidates'] = decision['candidates'][:requested]
                    else:
                        decision.pop('candidates', None)
        return selected
    # Analysis expands only the independently measured population. The winner
    # remains the exact geometry argmax below, regardless of any other channel.
    selected = geo_scores.max(1)[1]
    sorted_order = torch.argsort(geo_scores, dim=1, descending=True, stable=True)
    # Production selection uses ``max`` (first index on ties). Force that exact
    # winner to the front and keep original proposal order for every remaining tie.
    order = torch.stack([
        torch.cat((selected[b:b + 1], sorted_order[b][sorted_order[b] != selected[b]]))
        [:measured_count]
        for b in range(B)
    ])
    rows = []
    analysis_rows = []
    texture_by_original = torch.full_like(geo_scores, float('nan'))
    reference_errors = (_candidate_reference_errors(pred_rs, pred_ts, appe, radius)
                        if analysis_on else [None] * B)

    for b in range(B):
        sel = order[b]
        R = pred_rs[b][sel]
        t = pred_ts[b][sel].reshape(-1, 1, 3)
        pm = appe['dense_pm'][b][::stride]
        fm = torch.nn.functional.normalize(appe['dense_fm'][b][::stride], dim=1)
        po = appe['dense_po'][b]
        fo = torch.nn.functional.normalize(appe['dense_fo'][b], dim=1)
        obj = torch.einsum('kji,knj->kni', R, pm.unsqueeze(0) - t)
        nn = torch.cdist(obj, po.unsqueeze(0).expand(obj.size(0), -1, -1)).argmin(2)
        nn_distance = (torch.linalg.norm(obj - po[nn], dim=2)
                       if diag.get('enabled') else None)
        texture = torch.einsum('nd,knd->kn', fm, fo[nn]).mean(1)
        texture_by_original[b, sel] = texture
        color = None
        if have_col:
            cm = _whiten(appe['dense_cm'][b][::stride])
            co = _whiten(appe['dense_co'][b])
            color = (cm.unsqueeze(0) * co[nn]).mean((1, 2))

        geo_np = geo_scores[b][sel].detach().cpu().numpy()
        tex_np = texture.detach().cpu().numpy()
        col_np = None if color is None else color.detach().cpu().numpy()
        rank_geo_order = np.argsort(-geo_np, kind='stable')
        rank_tex_order = np.argsort(-tex_np, kind='stable')
        rank_geo = {int(k): int(i) for i, k in enumerate(rank_geo_order)}
        rank_texture = {int(k): int(i) for i, k in enumerate(rank_tex_order)}
        proposal_ids = appe.get('_proposal_ids')
        proposal_np = (sel.detach().cpu().numpy() if proposal_ids is None else
                       proposal_ids[b][sel].detach().cpu().numpy())

        texture_max_rank = ver.get('texture_max_rank')
        texture_min_score = ver.get('texture_min_score')
        texture_checks = []
        if texture_max_rank is not None:
            texture_checks.append(rank_texture[0] <= int(texture_max_rank))
        if texture_min_score is not None:
            texture_checks.append(float(tex_np[0]) >= float(texture_min_score))
        texture_status = ('diagnostic' if not texture_checks else
                          ('pass' if all(texture_checks) else 'fail'))

        selected_shape = None
        if shape is not None:
            k0 = int(sel[0])
            ratio = float(shape['size_ratio'][b, k0].item())
            iou = float(shape['mask_iou'][b, k0].item())
            coverage = float(shape['coverage'][b, k0].item())
            valid = bool(shape['projection_valid'][b, k0].item())
            size_limit = float(ver.get('size_ratio_max', 1.0))
            size_pass = valid and ratio <= size_limit
            iou_min = ver.get('iou_min')
            iou_status = ('unavailable' if not valid else 'diagnostic' if iou_min is None else
                          ('pass' if iou >= float(iou_min) else 'fail'))
            selected_shape = {
                'status': 'pass' if size_pass else 'fail',
                'reason': None if size_pass else ('projection_invalid' if not valid else
                                                  'render_too_small'),
                'size_ratio': round(ratio, 5), 'size_ratio_max': size_limit,
                'mask_iou': round(iou, 5), 'iou_status': iou_status,
                'iou_min': None if iou_min is None else float(iou_min),
                'coverage': round(coverage, 5),
                'observed_mask_area_px': int(shape['observed_mask_area_px'][b].item()),
                'observed_bbox_area_px': round(float(shape['observed_bbox_area_px'][b].item()), 2),
                'rendered_bbox_area_px': round(
                    float(shape['rendered_bbox_area_px'][b, k0].item()), 2),
                'rendered_mask_area_px': int(shape['rendered_mask_area_px'][b, k0].item()),
            }

        cands = None
        if dump and info is not None:
            keep = sorted(set(rank_geo_order[:dump].tolist()) |
                          set(rank_tex_order[:dump].tolist()) | {0})
            scale = 1.0 if radius is None else float(radius[b])
            Rk = R.detach().cpu().numpy()
            tk = (pred_ts[b][sel] * scale).detach().cpu().numpy()
            cands = []
            for k in keep:
                candidate = {
                    'R': [[round(float(v), 6) for v in row] for row in Rk[k]],
                    't_mm': [round(float(v) * 1000.0, 3) for v in tk[k]],
                    'proposal6000_index': int(proposal_np[k]),
                    'geo': round(float(geo_np[k]), 5),
                    'texture_score': round(float(tex_np[k]), 5),
                    'color_score': None if col_np is None else round(float(col_np[k]), 5),
                    'rank_geo': rank_geo[int(k)],
                    'rank_texture': rank_texture[int(k)],
                    'geometry_selected': bool(k == 0),
                }
                if shape is not None:
                    original = int(sel[k])
                    ratio = float(shape['size_ratio'][b, original].item())
                    valid = bool(shape['projection_valid'][b, original].item())
                    candidate['shape'] = {
                        'projection_valid': valid,
                        'size_ratio': round(ratio, 5),
                        'size_verified': bool(valid and ratio <= float(ver.get('size_ratio_max', 1.0))),
                        'mask_iou': round(float(shape['mask_iou'][b, original].item()), 5),
                        'coverage': round(float(shape['coverage'][b, original].item()), 5),
                        'observed_bbox_area_px': round(
                            float(shape['observed_bbox_area_px'][b].item()), 2),
                        'rendered_bbox_area_px': round(
                            float(shape['rendered_bbox_area_px'][b, original].item()), 2),
                    }
                cands.append(candidate)

        pointwise = None
        if diag_topn and info is not None:
            evidence_keep = sorted(set(rank_geo_order[:diag_topn].tolist()) |
                                   set(rank_tex_order[:diag_topn].tolist()) | {0})
            rgb_choose = appe.get('rgb_choose')
            coarse_idx = appe.get('coarse_fps_idx')
            geo_uv = (None if rgb_choose is None or coarse_idx is None else
                      rgb_choose[b][coarse_idx[b].long()])
            pointwise = _pointwise_evidence(
                pm, fm, (_whiten(appe['dense_cm'][b][::stride]) if have_col else None),
                po, fo, (_whiten(appe['dense_co'][b]) if have_col else None),
                nn, nn_distance,
                (None if rgb_choose is None else rgb_choose[b][::stride]),
                appe['_geo_point_distance'][b][sel], appe['_geo_point_valid'][b], geo_uv,
                (1.0 if radius is None else float(radius[b])), evidence_keep,
                rank_geo, rank_texture, diag_stride)

        top_texture = int(rank_tex_order[0])
        row = {
            'selection_method': 'geometry_only',
            'selected_geo_rank': 0,
            'selected_proposal6000_index': int(proposal_np[0]),
            'texture': {
                'status': texture_status,
                'score': round(float(tex_np[0]), 5),
                'rank': int(rank_texture[0]),
                'candidate_count': int(measured_count),
                'margin_to_texture_top1': round(float(tex_np[0] - tex_np[top_texture]), 5),
                'max_rank': texture_max_rank,
                'min_score': texture_min_score,
            },
            'shape': (selected_shape or {'status': 'unavailable',
                                         'reason': 'projection_metadata_missing'}),
        }
        if shape is not None:
            crop_values = appe['crop_bbox_yxyx'][b].detach().cpu().tolist()
            row['shape_mask'] = {
                'size': shape['mask_size'],
                'rle': shape['observed_mask_rle'][b],
                'bbox_224': shape['observed_bbox_224'][b],
                'crop_bbox_yxyx': [round(float(v), 3) for v in crop_values],
                'point_splat_radius_px': shape['point_splat_radius_px'],
                'provenance': 'depth-valid radius-inlier ISM mask; projected CAD is point-splat proxy',
            }
        if cands:
            row['cands'] = cands
        if pointwise:
            row['pointwise'] = pointwise
        rows.append(row)

        if analysis_on:
            finite_geo = geo_np[np.isfinite(geo_np)]
            score_min = float(np.min(finite_geo)) if finite_geo.size else math.nan
            score_max = float(np.max(finite_geo)) if finite_geo.size else math.nan
            span = score_max - score_min
            error = reference_errors[b]
            analysis_scale = 1.0 if radius is None else float(radius[b])
            analysis_rotations = R.detach().cpu().numpy()
            analysis_translations = (
                pred_ts[b][sel] * analysis_scale).detach().cpu().numpy()
            candidates = []
            for k in range(measured_count):
                original = int(sel[k])
                projection_valid = (None if shape is None else
                                    bool(shape['projection_valid'][b, original].item()))
                mask_iou = (None if not projection_valid else _rounded_finite(
                    shape['mask_iou'][b, original].item(), 6))
                geometry_score = _rounded_finite(geo_np[k], 8)
                texture_score = _rounded_finite(tex_np[k], 8)
                candidate = {
                    'rank_geo': int(k),
                    'index300': original,
                    'proposal6000_index': int(proposal_np[k]),
                    'R': [[round(float(value), 6) for value in matrix_row]
                          for matrix_row in analysis_rotations[k]],
                    't_mm': [round(float(value) * 1000.0, 3)
                             for value in analysis_translations[k]],
                    'geometry_score': geometry_score,
                    'geometry_score_detection_normalized': (
                        None if geometry_score is None or not np.isfinite(span) or span <= 0.0
                        else _rounded_finite((geo_np[k] - score_min) / span, 8)),
                    'texture_score': texture_score,
                    'mask_iou': mask_iou,
                    'projection_valid': projection_valid,
                    'missing_reason': ('projection_metadata_or_mask_unavailable'
                                       if shape is None else
                                       'invalid_projection' if not projection_valid else None),
                    'geometry_selected': bool(original == int(selected[b].item())),
                }
                candidate_valid = (error is None or bool(error['pose_valid'][original].item()))
                candidate['candidate_valid'] = candidate_valid
                if error is None:
                    candidate.update({'gt_status': 'unavailable', 'correct': None,
                                      'rotation_error_deg': None,
                                      'translation_error_mm': None})
                elif not candidate_valid:
                    candidate.update({'gt_status': 'available', 'correct': None,
                                      'rotation_error_deg': None,
                                      'translation_error_mm': None,
                                      'missing_reason': 'invalid_candidate_pose'})
                else:
                    candidate.update({
                        'gt_status': 'available',
                        'correct': bool(error['correct'][original].item()),
                        'rotation_error_deg': _rounded_finite(
                            error['rotation_error_deg'][original].item(), 5),
                        'translation_error_mm': _rounded_finite(
                            error['translation_error_mm'][original].item(), 5),
                    })
                candidates.append(candidate)
            analysis_rows.append({
                'schema_version': 1,
                'selection_method': 'geometry_only',
                'candidate_count': int(measured_count),
                'selected_index300': int(selected[b].item()),
                'correctness': _correctness_metadata(diag, (appe.get('names') or [None] * B)[b]),
                'channels': {
                    'geometry_score': {'direction': 'higher_is_better', 'kind': 'PEM geometry'},
                    'geometry_score_detection_normalized': {
                        'direction': 'higher_is_better', 'normalization': 'per-detection min-max'},
                    'texture_score': {'direction': 'higher_is_better', 'kind': 'independent'},
                    'mask_iou': {'direction': 'higher_is_better', 'kind': 'independent'},
                },
                'candidates': candidates,
            })

    if info is not None:
        info['verify'] = rows
        if analysis_rows:
            existing = info.get('score_analysis') or [{} for _ in range(B)]
            info['score_analysis'] = [
                {**existing[b], 'stage300': analysis_rows[b]} for b in range(B)
            ]
        physical_t = pred_ts.clone()
        if radius is not None:
            physical_t = physical_t * radius.reshape(-1, 1, 1)
        _attach_explorer_v2(info, pred_rs, physical_t, geo_scores,
                            texture_by_original, shape, appe, None, selected)
    return selected


def validate_refined_poses(pred_rs, pred_ts_m, pose_scores, appe, verify_rows):
    """Recompute 8192-point Mask and deep-feature Texture for fine poses in place."""
    ver = appe.get('verify') or {}
    if not ver.get('enabled', False):
        return verify_rows
    radius = appe.get('radius')
    normalized_t = (pred_ts_m if radius is None else
                    pred_ts_m / radius.reshape(-1, 1).clamp_min(1e-8))
    shape = _candidate_shape_metrics(
        pred_rs[:, None], normalized_t[:, None], appe.get('_projection_model_pts'), appe)
    stride = max(1, int(appe.get('stride', 1)))
    texture = torch.full((pred_rs.size(0),), float('nan'), device=pred_rs.device)
    for b in range(pred_rs.size(0)):
        pm = appe['dense_pm'][b][::stride]
        fm = F.normalize(appe['dense_fm'][b][::stride], dim=1)
        po = appe['dense_po'][b]
        fo = F.normalize(appe['dense_fo'][b], dim=1)
        obj = torch.einsum('ji,nj->ni', pred_rs[b], pm - normalized_t[b])
        nearest = torch.cdist(obj[None], po[None]).squeeze(0).argmin(1)
        texture[b] = torch.einsum('nd,nd->n', fm, fo[nearest]).mean()

    mask_min = float(ver.get('mask_iou_min', ver.get('iou_min', 0.420998)))
    texture_min = float(ver.get('texture_min_score', 0.449562))
    flat_scores = None if pose_scores is None else pose_scores.reshape(-1)
    for b, row in enumerate(verify_rows):
        R = pred_rs[b]
        t = pred_ts_m[b]
        fine_ok = bool(torch.isfinite(R).all().item() and torch.isfinite(t).all().item())
        fine_ok = fine_ok and bool((torch.linalg.det(R) > 0.99).item())
        if flat_scores is not None:
            fine_ok = fine_ok and bool(torch.isfinite(flat_scores[b]).item())
        projection_ok = bool(shape is not None and shape['projection_valid'][b, 0].item())
        iou = (float(shape['mask_iou'][b, 0].item()) if projection_ok else float('nan'))
        tex = float(texture[b].item())
        mask_pass = projection_ok and math.isfinite(iou) and iou >= mask_min
        texture_pass = math.isfinite(tex) and tex >= texture_min
        coarse_accepted = bool(row.get('accepted', False))
        accepted = coarse_accepted and fine_ok and mask_pass and texture_pass
        reason = row.get('rejection_reason')
        if coarse_accepted and not accepted:
            if not fine_ok:
                reason = 'fine_refinement_failed'
            elif not mask_pass:
                reason = 'final_mask_below_threshold'
            else:
                reason = 'final_texture_below_threshold'
        row['accepted'] = accepted
        row['rejection_reason'] = reason
        row['fine'] = {
            'valid': fine_ok,
            'projection_valid': projection_ok,
            'mask_iou': _rounded_finite(iou, 6) if math.isfinite(iou) else None,
            'mask_pass': mask_pass,
            'texture_score': _rounded_finite(tex, 8) if math.isfinite(tex) else None,
            'texture_pass': texture_pass,
            # A low mask score alone is not an anchor invalidation signal. This pose can
            # still be compared by the anchor shadow validator when fine+texture are valid.
            'anchor_shadow_valid': fine_ok and texture_pass,
        }
    return verify_rows


def appearance_rerank(pred_rs, pred_ts, geo_scores, appe, info=None):
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
    diag = appe.get('diagnostic') or {}
    diag_on = bool(diag.get('enabled', False))
    diag_topn = max(0, int(diag.get('topn', 10))) if diag_on else 0
    dump = max(dump, diag_topn)
    diag_stride = max(1, int(diag.get('evidence_stride', 8)))
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
        nn_distance = (torch.linalg.norm(obj - po[nn], dim=2) if diag_on else None)
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

        if not on and not diag_on:
            idx_out.append(sel[torch.argmax(s)])
            continue

        # 승자는 검증이 꺼져 있을 때와 **같은 연산**(torch.argmax)으로 뽑는다. 거의 같은
        # 점수의 후보가 흔해서(동점 구간이 이 단계의 본질이다) 동점 처리 규칙이 다른
        # numpy 로 옮기면 이유 없는 차이가 생길 수 있다. 파리티는 실측으로 확인했다:
        # w_col=0 · geo_guard=0 이면 검출 2075건의 R·t·score 가 OFF 와 완전히 같다.
        if guard > 0:
            elig_t = g >= guard * g.max()
            elig_t[0] = True                              # 기하 1등은 언제나 후보
            win = int(torch.argmax(s.masked_fill(~elig_t, float('-inf'))))
        else:
            elig_t = torch.ones_like(g, dtype=torch.bool)
            win = int(torch.argmax(s))

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
        pointwise = None
        if diag_topn and info is not None:
            # 결합점수 상위 N과 기하 상위 N의 합집합만 보존한다. 전체 점은
            # evidence_stride로 축약하므로 브라우저와 산출물 크기를 통제할 수 있다.
            rg = np.argsort(-g_np)
            rs = np.argsort(-s_e)
            evidence_keep = sorted(set(rg[:diag_topn].tolist()) |
                                   set(rs[:diag_topn].tolist()) | {0, win})
            rank_g = {int(k): int(i) for i, k in enumerate(rg)}
            rank_s = {int(k): int(i) for i, k in enumerate(rs)}
            rgb_choose = appe.get('rgb_choose')
            geo_distance = appe['_geo_point_distance'][b][sel]
            geo_valid = appe['_geo_point_valid'][b]
            coarse_idx = appe.get('coarse_fps_idx')
            geo_uv = (None if rgb_choose is None or coarse_idx is None else
                      rgb_choose[b][coarse_idx[b].long()])
            pointwise = _pointwise_evidence(
                pm, fm, (_whiten(appe['dense_cm'][b][::stride]) if w_col else None),
                po, fo, (_whiten(appe['dense_co'][b]) if w_col else None),
                nn, nn_distance,
                (None if rgb_choose is None else rgb_choose[b][::stride]),
                geo_distance, geo_valid, geo_uv,
                (1.0 if radius is None else float(radius[b])), evidence_keep,
                rank_g, rank_s, diag_stride)
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
                **({'pointwise': pointwise} if pointwise else {}),
            })
    if info is not None and rows:
        info['verify'] = rows
    return torch.stack(idx_out)


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
    diagnostic = ((appe or {}).get('diagnostic') or {})
    diag_on = bool(diagnostic.get('enabled', False))
    score_analysis_on = bool(diag_on and
                             (diagnostic.get('score_analysis') or {}).get('enabled'))
    stage6000 = (_reference_stage_summary(pred_rs, pred_ts.squeeze(2), appe,
                                          'stage6000', appe.get('radius'))
                 if diag_on and info is not None else None)
    if score_analysis_on and info is not None:
        aggregate = _stage6000_score_analysis(
            pred_rs, pred_ts.squeeze(2), dis, appe, appe.get('radius'))
        info['score_analysis'] = [{'stage6000': aggregate[b]} for b in range(B)]
    proposal_ids = torch.topk(dis, n_proposal2, dim=1, largest=False)[1]
    pred_rs = torch.gather(pred_rs, 1, proposal_ids.reshape(B,n_proposal2,1,1).repeat(1,1,3,3))
    pred_ts = torch.gather(pred_ts, 1, proposal_ids.reshape(B,n_proposal2,1,1).repeat(1,1,1,3))
    if diag_on and info is not None:
        stage300 = _reference_stage_summary(pred_rs, pred_ts.squeeze(2), appe,
                                            'stage300', appe.get('radius'))
        info['diagnostic'] = [
            {'schema_version': 1, 'stage6000': stage6000[b], 'stage300': stage300[b]}
            for b in range(B)
        ]

    # pose selection
    transformed_pts = (pts1.unsqueeze(1) - pred_ts) @ pred_rs
    transformed_pts = transformed_pts.reshape(B*n_proposal2, -1, 3)
    dis = torch.sqrt(pairwise_distance(transformed_pts, expand_model_pts))
    dis = dis.min(2)[0].reshape(B, n_proposal2, -1)
    scores = weights1.unsqueeze(1).sum(2) / ((dis * weights1.unsqueeze(1)).sum(2) + + 1e-8)
    # Geometry order itself is immutable. Enabled verification applies the sequential
    # Mask -> Texture -> convergence policy and returns one actual proposal index.
    if diag_on:
        # 실제 기하 점수의 분모에 쓰인 196개 최근접거리와 유효 마스크를
        # appearance_rerank의 상위-N 계측기로 전달한다. 진단 OFF에서는 복사/추가가 없다.
        appe = dict(appe, _geo_point_distance=dis, _geo_point_valid=weights1)
    idx = scores.max(1)[1]
    if appe is not None:
        appe = dict(appe, _proposal_ids=proposal_ids,
                    _projection_model_pts=appe.get('projection_model', model_pts))
        idx = independent_candidate_verify(
            pred_rs, pred_ts.squeeze(2), scores, appe, info)
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
