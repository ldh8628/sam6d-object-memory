# Bear `rank_geo=42` 증거 누락 조사

- 조사 시각: 2026-08-23 (Asia/Seoul)
- 입력: `output/longcircle2_sam_self_slam_diagnostic/dump.jsonl`
- 최초 Bear detection: `stamp_ns=1785831508280933376`, `frame_index=62`

## 확인 결과

- `verify.cands`에는 100개 후보가 있으며 `rank_geo=42`도 존재한다.
- 현재 실행의 `verify.pointwise`는 기하/텍스처 상위 집합 18개만 보존했고 `rank_geo=42`는 포함하지 않았다.
- 현재 게시 명령은 `--render-topn 10`이므로 builder도 rank 0–9만 시각 asset으로 만들었다.

따라서 #42가 보이지 않는 직접 원인은 후보 pose/score 데이터 누락이 아니라 게시 제한 `render-topn=10`이다. 다만 #42의 정확한 geometry/texture heatmap까지 만들려면 보고서 옵션만 100으로 바꾸는 것으로는 부족하며, 원 진단을 `--pem-diagnostic-topn 100`으로 다시 실행해 pointwise 근거도 수집해야 한다. 새 builder는 `render-topn=100` 게시 시 각 후보의 pointwise asset이 없으면 원자 교체 전에 실패하도록 검증한다.

## 복구 및 Shadow 재현 결과

- 새 진단은 2,979 detection 모두에 기하 순위 0–99 pointwise 근거를 수집했다.
- 선택 parity는 2,979/2,979 행에서 geometry-selected proposal, R, t_mm이 허용오차 0으로 일치했다.
- 게시 리포트에는 297,900개 상위-100 후보와 1,492,500개 evidence asset(상위-100 밖 Shadow 7행의 pose/mask proxy 포함)이 있으며 순위·pointwise·공유 mask 위반은 모두 0건이다.
- Tune만으로 선택된 군집 임계값은 계획대로 `20°/25mm`다. Tune은 대체 22건 중 정답 20건, 전체 수용 정확도 420/466(90.13%), coverage 68.83%, 정답→오답 대체 0건이다.
- 고정 임계값의 실제 Holdout 결과는 대체 7건 중 정답 7건, 전체 수용 352건 중 정답 325건(92.33%), coverage 70.82%다.

계획에 적힌 Holdout 예상치 `10/9`, `355/327`, `71.4%`는 최종 full-stage300 데이터에서 재현되지 않았다. 차이를 만드는 세 개의 연속 `saffron` 사례(`frame_index=907,916,925`)는 점수를 통과하지만 표준 SO(3) geodesic 회전 거리가 10.5–18.8°, 이동 거리가 6–20mm여서 명시된 `≤20° AND ≤25mm` 규칙상 Top-1 군집에 포함된다. 대칭을 전부 끈 결과는 7/6이며, 여기에 이 세 군집 후보를 잘못 포함할 때만 10/9가 되므로 예상치는 서로 다른 거리/대칭 계산이 혼합된 것으로 판단했다. 결과 수를 맞추기 위해 규칙을 왜곡하지 않고 명시된 symmetry-aware 거리와 AND 경계를 적용했다.

Chrome deep link `?frame=62&object=Bear&rank_geo=42`에서 Bear #42, proposal 3642, R/t 및 pose·geometry·texture·rendered mask·mask overlap을 확인했다.
