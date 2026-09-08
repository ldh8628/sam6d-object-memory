---
status: done
baseline_commit: NO_VCS
specLoopIteration: 1
---

# PEM 정답 후보 점수 분포 및 독립 필터 기준 분석 명세

## 1. 목적과 현재 증거

longcircle2 SAM 전용 RGB-D 데이터에서 객체별 정답 포즈 후보와 오답 포즈 후보의 점수 분포를 시각화하고, 6000개 또는 300개 후보 단계에서 정답 후보를 보존하면서 오답 후보를 제거할 수 있는 기준을 찾는다.

기존 진단 결과는 각 detection의 상위 100개 후보에만 `geo`, `texture_score`, `mask_iou`가 기록됐다. 이번 구현은 geometry-only 선택을 바꾸지 않은 채 분석 모드에서 300개 후보 전체의 세 점수를 기록하도록 계측을 확장했다. 6000개 전체에는 PEM 기하·텍스처·IoU가 계산되지 않고 3-point 초기 잔차만 존재한다.

최종 stage recall은 GT 사용 가능 evaluation detection 1,734개 중 6000단계 O 1,415개(81.6%), 300단계 O 1,326개(76.5%)다. 즉 6000→300에서 정답 후보가 존재하던 89개 detection이 전부 탈락한다. 따라서 현재 후보 생성·축소를 유지한 채 점수 필터만 추가해서는 전체 90%에 도달할 수 없다. 이 손실과 300단계 이후의 점수별 필터 성능을 분리해 분석한다.

## 2. 고정된 분석 정의

- 정답 후보: 신뢰 가능한 pseudo-GT에 대해 symmetry-aware 회전 오차 30도 이하이면서 평행이동 오차 100 mm 이하인 후보.
- 정답 라벨 사용 객체: `Bear`, `Dinosaur`, `Mugcup_high`, `Sauce_high`, `Sikhye_high`, `milk`, `saffron` 7개.
- `Febreze_high`, `choco_hazelnut_high`는 GT 신뢰 부족으로 정답/오답 분포 및 임계값 학습에서 제외한다. UI에는 “GT 신뢰 부족”과 라벨 없는 전체 후보 분포만 표시한다.
- pseudo-GT reference member 102개 프레임은 임계값 추정과 성능 평가에서 제외한다.
- 6000단계 점수는 3-point proposal residual이며, PEM 기하 점수라고 부르지 않는다.
- 300단계에서는 300개 전부에 대해 PEM 기하 점수, 텍스처 비교 점수, 2D Mask IoU를 분석용으로 계산한다.
- 점수 채널은 서로 독립된 필터다. 가중합, z-score 합산, 학습된 결합 점수, 채널 간 재순위화는 사용하지 않는다.
- 기존 geometry-only 300→1 선택 결과는 분석 전후 완전히 동일해야 한다.

## 3. 구현 범위

### 3.1 분석용 계측

- 6000개 후보의 초기 잔차와 정답 여부를 메모리에서 집계해 객체별 histogram, quantile, stage survival 통계를 저장한다. 약 1,800만 후보의 원본 행을 전부 dump하지 않는다.
- 300개 후보 전부에 대해 기하·텍스처·IoU와 GT 오차, projection 유효성, frame/object 식별자를 compact record로 저장한다.
- 분석 계측을 끄면 기존 실행 경로와 출력이 바뀌지 않아야 한다.
- 점수 누락, invalid projection, 빈 mask는 별도 결측 범주로 집계하며 임의의 0점으로 섞지 않는다.

### 3.2 분포와 임계값 분석

각 신뢰 객체 및 각 독립 채널에 대해 다음을 생성한다.

- 정답/오답 후보 histogram과 ECDF.
- 기하 점수는 raw와 detection 내 정규화 값 모두 표시해 객체·프레임별 scale 변동을 확인한다.
- 텍스처 점수와 IoU는 raw 분포, quantile, 정답/오답 겹침 구간을 표시한다.
- threshold sweep으로 정답 후보 보존율, 오답 후보 제거율, detection당 잔존 후보 수, frame-level recall을 표시한다.
- 6000→300→필터 후 단계별 정답 후보 생존 수와 탈락 수를 표시한다.

시간 순서를 유지한 blocked split을 사용한다. 앞 절반의 evaluation frame은 threshold 탐색, 뒤 절반은 holdout 평가에 사용한다. 기본 후보 기준은 탐색 구간에서 정답 후보의 최소 95%를 보존하는 방향의 객체별·채널별 단일 threshold다. holdout에서 다음을 별도로 보고한다.

- correct-candidate retention
- incorrect-candidate rejection
- 정답 후보가 필터 전 존재한 detection 중 하나 이상 보존되는 frame-level recall
- 필터 후 평균/중앙 후보 수

정답 표본이 부족하거나 holdout에 정답 후보가 없으면 threshold를 제안하지 않고 `insufficient_data`로 표시한다. 이 결과는 동일 시퀀스에서 만든 self-SLAM pseudo-GT 기반 탐색 결과임을 명시하며 독립 데이터 일반화 성능으로 표현하지 않는다.

### 3.3 재사용 가능한 HTML

`output/pem_score_distributions/index.html`을 생성하고 다른 데이터의 compact analysis JSON을 교체해도 사용할 수 있게 한다.

- 9개 객체를 한 화면에서 선택/비교.
- 신뢰 객체 7개는 세 점수의 정답/오답 분포와 threshold sweep을 표시.
- 신뢰 부족 객체 2개는 GT 기반 그래프 대신 명확한 unavailable 상태와 라벨 없는 분포 표시.
- 객체별 threshold 표에 tune/holdout 지표와 표본 수를 함께 표시.
- 6000, 300, 필터 후의 단계별 생존 그래프 표시.
- 기존 `output/pem_explorer/index.html`에서 분석 페이지로 이동할 수 있는 링크 추가.
- 그래프 툴팁에 bin/threshold, 정답·오답 수, 결측 수를 표시한다.

## 4. 경계 조건

### Always

- 7개 trusted pseudo-GT와 evaluation frame만 threshold 라벨로 사용한다.
- 정답 정의, symmetry 처리, 단위와 split 정보를 결과 JSON/HTML에 기록한다.
- 객체별·채널별 결과와 전체 결과를 구분한다.
- 개별 독립 필터 및 독립 필터의 AND 교집합은 분석할 수 있지만, 합성 점수로 만들지 않는다.

### Ask First

- 제안 threshold를 realtime 실행에 활성화하기.
- 최종 geometry-selected pose를 reject하거나 다른 후보로 승격하기.
- 정답 오차 기준, GT 신뢰 객체, split 정책을 변경하기.

### Never

- 신뢰 부족 GT로 정답/오답 분포나 threshold를 학습하지 않는다.
- reference member를 독립 평가 표본으로 사용하지 않는다.
- 6000개에 PEM 기하·텍스처·IoU가 모두 계산되었다고 표현하지 않는다.
- tune 결과를 holdout 결과로 재사용하거나 weighted composite score를 만들지 않는다.

## 5. 주요 파일과 작업

- `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py`: 분석 모드의 6000 집계와 300개 검증 점수 수집. 기존 선택 로직 불변.
- `temp/verify_eval.py`: 분석 모드 설정 및 compact artifact 출력.
- `tools/analyze_pem_candidate_score_distributions.py`: split, 분포, threshold sweep, 지표 산출 및 HTML data 생성.
- `tools/pem_score_report/`: 재사용 가능한 HTML/CSS/JS 템플릿.
- `output/pem_score_distributions/`: longcircle2 결과.
- `tests/`: 정답 라벨, split 누수 방지, threshold 방향, 결측 처리, selection 불변, HTML schema 테스트.

## 6. 인수 조건

1. 9개 객체 패널이 있으며 trusted 7개에는 정답/오답 기하·텍스처·IoU 분포가 모두 표시된다.
2. untrusted 2개는 GT 기반 결론이 불가능하다고 표시되고 threshold가 생성되지 않는다.
3. 6000단계는 initial residual, 300단계는 세 검증 점수라는 차이가 UI와 데이터에 명시된다.
4. 객체별·채널별 threshold와 tune/holdout의 보존율, 제거율, frame recall, 잔존 후보 수가 표시된다.
5. 6000→300에서 정답 후보가 탈락한 detection 수를 포함한 stage attrition이 표시된다.
6. 분석 모드를 켜고 껐을 때 기존 geometry-selected 후보 index와 최종 pose가 동일하다.
7. threshold는 제안 상태로만 저장되며 realtime 필터에는 적용되지 않는다.
8. compact JSON을 다른 결과로 바꿔도 같은 HTML을 재사용할 수 있다.
9. 자동 테스트와 브라우저 렌더링 검증을 통과한다.

## 7. 검증 계획

- 단위 테스트: correctness label, symmetry, blocked split, score 방향, quantile threshold, 결측값, insufficient-data.
- 회귀 테스트: 분석 모드 on/off의 300→1 index 및 pose 일치.
- 데이터 감사: 객체별 후보 수, reference 제외 수, invalid score 수, stage count 일관성.
- longcircle2 전체 실행 후 생성 JSON과 HTML schema 검사.
- 브라우저에서 9개 객체 전환, 그래프/툴팁, threshold 표, 기존 explorer 링크 확인.

## 8. 구현 순서

1. selection 불변 분석 계측과 테스트를 추가한다.
2. longcircle2에서 6000 aggregate 및 300개 전체 점수 artifact를 생성한다.
3. 분포·threshold 분석기를 구현하고 tune/holdout 지표를 산출한다.
4. 재사용 HTML을 만들고 기존 explorer landing에 연결한다.
5. 회귀·데이터 감사·브라우저 검증 후 수치와 해석을 보고한다.

## 9. 구현 및 검증 결과

- longcircle2 전체 2,979 detection에 대해 300개 후보의 기하·텍스처·IoU 분석 artifact를 생성했다.
- GT 사용 가능 evaluation detection 1,734개에서 6000 정답 포함 1,415개(81.6%), 300 정답 포함 1,326개(76.5%), 6000→300 손실 89개를 확인했다.
- 7개 trusted 객체는 정답/오답 분포와 time-blocked tune/holdout threshold를 생성했다. `Sauce_high`는 tune 표본 부족으로 `insufficient_data`, untrusted 2개는 라벨 없는 분포만 생성했다.
- proposal-only이며 realtime threshold는 활성화하지 않았다. geometry-only 선택 결과는 기준 run과 2,979건 모두 후보 index·R·t가 완전히 동일했다.
- SAM-6D conda 환경 전체 테스트 120개가 통과했다. Headless Chrome에서 9개 객체 URL 전환과 최신 ceiling 표시를 확인했다.
- 상세 수치와 적용 판단은 `output/pem_score_distributions/findings.md`, 감사 증적은 `output/longcircle2_sam_pem_score_analysis/selection-parity.json`에 기록했다.

## Suggested Review Order

1. `output/pem_score_distributions/index.html`에서 객체별 paired histogram, ECDF, threshold sweep, stage survival을 확인한다.
2. `output/pem_score_distributions/findings.md`에서 90% ceiling과 객체별 적용 판단을 검토한다.
3. `tools/analyze_pem_candidate_score_distributions.py`의 correctness, split, threshold 및 provenance 검증을 확인한다.
4. `sam6d_master/SAM-6D/Pose_Estimation_Model/utils/model_utils.py`와 `realtime/sam6d_core.py`에서 계측이 기존 geometry-only 선택을 변경하지 않는지 확인한다.
5. `tests/test_pem_score_distributions.py`, `tests/test_pem_candidate_verification.py`와 selection parity 증적을 확인한다.

## 10. 최종 리뷰 처리

- Blind 및 acceptance 리뷰는 남은 High/Medium 결함 없이 통과했다.
- 파일 여러 개를 순차 `os.replace`하는 동안 프로세스가 중단되면 세대가 섞일 수 있다는 이론적 race는 현재 산출물 영향 없음으로 분류했다. 이 도구는 로컬 offline 단일 publisher로 실행되고, 현재 `analysis.json`/`analysis-data.js`/HTML 자산 일관성과 브라우저 렌더를 재검증했다. versioned-directory pointer 전환은 동시 서비스가 필요해질 때 적용한다.
- 최종 input 검증과 publication 사이의 TOCTOU도 현재 hash가 모두 일치하고 producer가 artifact를 atomic replace하는 단일 실행 환경이므로 현재 영향 없음으로 분류했다. 외부 동시 writer를 허용하는 배포 형태로 바뀌면 immutable snapshot 또는 lock을 추가한다.
