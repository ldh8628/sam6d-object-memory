# Deferred Work

작업 중 분리되어 보류된 목표를 추적한다. 메인 작업 완료 후 별도 스펙으로 진행한다.

## [2026-06-17] SLAM 비교 — 시각화 + 보고서 레이어  ✅ 완료 (2026-06-17)

**완료:** 사용자 요청으로 이어서 구현. 산출물:
- 16 PNG: 각 `output/<ds>/{trajectory_topview.png, trajectory_topview_keyframes.png}` (ORB·RTAB 통일 X–Z Top-View, Start=Red/End=Blue/궤적선/방향화살표/KeyFrame 별/범례). 생성: `slam_comparison_report/make_topviews.py`.
- `slam_comparison_report/slam_visual_comparison.md` 및 `.pdf` (4 dataset별 ORB vs RTAB 이미지 + 7개 분석 항목 + 요약표; PDF는 matplotlib PdfPages, NanumGothic 한글). 생성: `slam_comparison_report/build_report.py`.
- PNG 16장 + PDF 육안 검증 완료.

**원 스펙에서 분리:** `spec-slam-topview-comparison.md` (실행/추출 vs 시각화/보고서 분리, 사용자 선택 Split)

**보류된 산출물:**
- 통일된 Top-View 시각화 PNG (4 dataset × 2 SLAM × 2종 = 16장):
  - `<output>/<dataset>/trajectory_topview.png` (맵 + 궤적 + Start=Red/End=Blue + 진행방향 + 범례 + X–Z 축)
  - `<output>/<dataset>/trajectory_topview_keyframes.png` (+ KeyFrame 마커)
  - ORB·RTAB 동일 dataset은 축·투영·색·마커 규칙 통일
- 비교 보고서: `slam_comparison_report/slam_visual_comparison.md` 및 `.pdf`
  - 4 dataset별 ORB vs RTAB 이미지 나란히 임베드
  - 분석 항목: trajectory 일관성, drift, loop closure 유무, keyframe 분포, 경로 형태 차이, 시작-종료점 오차, 맵 생성 품질
  - PDF는 오프라인 생성(fpdf2/matplotlib PdfPages, 네트워크 비의존)

**선결 의존:** 메인 스펙의 실행 결과(각 `output/<dataset>/`의 trajectory.txt·keyframes.txt·map_points.pcd/dense_map.pcd)가 존재해야 함.

**참고 자산:** `rtabmap_ws/scripts/export_rtabmap_outputs.py`(top-view 플롯·PLY/PCD 파서), matplotlib는 conda `rtabmap` env에만 존재.

## [2026-06-17] 실행 스크립트 리뷰 — defer 항목 (robustness, 비차단)

step-04 adversarial 리뷰에서 나온 비차단 개선점. 현재 8개 실행 산출물은 모두 검증 완료(정상)이며 아래는 향후 견고성 개선용.

- **PCD `rgb` 필드 인코딩:** `run_rtabmap_four_bags.py`의 `write_ascii_pcd`는 rgb를 `TYPE U`(정수)로 기록(원본 `run_four_slam_bags.py` 관행). PCL 등 표준 소비자는 packed-float32(`TYPE F`)를 기대 → 색상 오독 가능. 기하(POINTS·좌표)는 정상. 색상 호환이 필요하면 packed-float로 변경.
- **RTAB 고정 sleep(8s/5s):** 노드 준비/큐 드레인을 고정 대기로 처리 → 콜드 캐시/고부하 시 초기 프레임 누락 또는 말미 keyframe 누락 가능. 노드 구독 준비성 폴링 + 마지막 처리 스탬프 감시로 대체하면 견고. (ORB는 이벤트 기반이라 무관.)
