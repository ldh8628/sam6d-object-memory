
## [2026-06-17] yolo-ism-multiobject 리뷰 중 발견 (defer)
- **yolo_ism.segment_box MobileSAM device=0 하드코딩**: `yolo_ism.py:255`가 `device=0 if device.startswith("cuda")`로 GPU 인덱스를 무시 → `--device cuda:1` 환경에서 DINOv2(cuda:1)와 segmentor(cuda:0) 디바이스 불일치 가능. 원본 yolo_ism.py 무수정 제약상 본 작업 범위 밖. 다중 GPU 사용 시 yolo_ism.py 측 수정 필요.
