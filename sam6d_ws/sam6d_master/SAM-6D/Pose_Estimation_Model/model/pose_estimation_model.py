import torch
import torch.nn as nn

from feature_extraction import ViTEncoder
from coarse_point_matching import CoarsePointMatching
from fine_point_matching import FinePointMatching
from transformer import GeometricStructureEmbedding
from model_utils import sample_pts_feats, get_chosen_pixel_feats


class Net(nn.Module):
    def __init__(self, cfg):
        super(Net, self).__init__()
        self.cfg = cfg
        self.coarse_npoint = cfg.coarse_npoint
        self.fine_npoint = cfg.fine_npoint

        self.feature_extraction = ViTEncoder(cfg.feature_extraction, self.fine_npoint)
        self.geo_embedding = GeometricStructureEmbedding(cfg.geo_embedding)
        self.coarse_point_matching = CoarsePointMatching(cfg.coarse_point_matching)
        self.fine_point_matching = FinePointMatching(cfg.fine_point_matching)

    def forward(self, end_points):
        dense_pm, dense_fm, dense_po, dense_fo, radius = self.feature_extraction(end_points)

        # 외형 재정렬(기본 OFF). cfg.appe_rerank 가 있을 때만 coarse 의 승자 선택에 관여한다.
        # 특징은 여기서 이미 계산된 것을 넘길 뿐이라 추가 연산이 없다.
        appe_cfg = getattr(self.cfg, 'appe_rerank', None)
        if appe_cfg and not self.training:
            d = dict(appe_cfg, dense_pm=dense_pm, dense_fm=dense_fm,
                     dense_po=dense_po, dense_fo=dense_fo)
            # 원색(RGB) 채널. 모델 쪽 색은 템플릿 캐시가 들고 오고(dense_co),
            # 관측 쪽 색은 이미 들어와 있는 crop 을 정규화만 되돌려 그 자리에서 뽑는다
            # (추가 자산·추가 이미지 로드 없음).
            ver = appe_cfg.get('verify') or {}
            if (ver.get('enabled') and float(ver.get('w_col', 0.0)) != 0.0
                    and end_points.get('dense_co') is not None):
                rgb = end_points['rgb']
                m = rgb.new_tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
                sd = rgb.new_tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
                d['dense_cm'] = get_chosen_pixel_feats(rgb * sd + m, end_points['rgb_choose'])
                d['dense_co'] = end_points['dense_co']
            d['radius'] = radius          # 후보 병진을 원래 크기로 되돌릴 때 쓴다
            # 마스크 비교 게이트(ch1). 설정은 appe_cfg 에 실려 오고, 텐서는 여기서
            # **새 dict** 에 담는다 — appe_cfg 를 그대로 mutate 하면 GPU 텐서가
            # self.cfg.appe_rerank 에 눌러앉아 누수 + verify_eval 의 설정 교체가 깨진다.
            # ⚠ d = dict(appe_cfg, ...) 라 설정 dict 가 **이미 복사돼 있다**. 켜지 않을 때
            #   지우지 않으면 텐서 없는 껍데기가 남아 게이트가 그걸 집어든다.
            mg_cfg = d.pop('mask_gate', None)
            if (mg_cfg and mg_cfg.get('enabled')
                    and end_points.get('mask_g') is not None
                    and end_points.get('K') is not None):
                d['mask_gate'] = dict(dict(mg_cfg), mask=end_points['mask_g'],
                                      bbox=end_points['bbox'], K=end_points['K'],
                                      radius=radius)
            # ch2 — Map 사전. 참조 자세는 추론 프로세스가 PEM 을 부르기 전에 만들어
            # 넣어 준다(그때만 T_map_cam 을 안다). 없으면 껍데기를 남기지 않는다.
            mp_cfg = d.pop('map_prior', None)
            if (mp_cfg and mp_cfg.get('enabled')
                    and end_points.get('map_ref_R') is not None):
                ver_cfg = appe_cfg.get('verify') or {}
                d['map_prior'] = dict(dict(mp_cfg), ref_R=end_points['map_ref_R'],
                                      ref_ok=end_points['map_ref_ok'],
                                      names=end_points.get('obj_names'),
                                      sym_axes=ver_cfg.get('symmetry_axes'),
                                      sym_step=ver_cfg.get('sym_step_deg', 10))
            if end_points.get('obj_names') is not None:
                d['names'] = end_points['obj_names']     # 대칭 선언 조회용
            end_points['appe_rerank'] = d

        # pre-compute geometric embeddings for geometric transformer
        bg_point = torch.ones(dense_pm.size(0),1,3).float().to(dense_pm.device) * 100

        sparse_pm, sparse_fm, fps_idx_m = sample_pts_feats(
            dense_pm, dense_fm, self.coarse_npoint, return_index=True
        )
        geo_embedding_m = self.geo_embedding(torch.cat([bg_point, sparse_pm], dim=1))

        sparse_po, sparse_fo, fps_idx_o = sample_pts_feats(
            dense_po, dense_fo, self.coarse_npoint, return_index=True
        )
        geo_embedding_o = self.geo_embedding(torch.cat([bg_point, sparse_po], dim=1))

        # coarse_point_matching
        end_points = self.coarse_point_matching(
            sparse_pm, sparse_fm, geo_embedding_m,
            sparse_po, sparse_fo, geo_embedding_o,
            radius, end_points,
        )

        # fine_point_matching
        end_points = self.fine_point_matching(
            dense_pm, dense_fm, geo_embedding_m, fps_idx_m,
            dense_po, dense_fo, geo_embedding_o, fps_idx_o,
            radius, end_points
        )

        return end_points

