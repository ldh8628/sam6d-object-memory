# Deferred work

## 2026-08-05 — SLAM runtime bundle review

- ORB3-SLAM 및 SAM-6D subscriber에 `SensorDataQoS`/best-effort 선택지를 제공해 다양한 카메라·bag publisher와의 호환성을 높인다.
- ORB3-SLAM, RTAB-MAP, SAM-6D bag 종료 시 queued frame을 drain한 뒤 결과를 저장하도록 종료 순서를 보강한다.
- SAM-6D 결과 pose에 입력 timestamp/frame을 유지하고, 결과 이미지 publish 여부를 GUI `imshow` 옵션과 분리한다.
- ORB3-SLAM의 `-march=native`를 선택 옵션으로 바꿔 다른 CPU에서 source build한 산출물의 이식성을 명확히 한다.
- ORB3-SLAM 선택적 trajectory visualization helper와 RTAB-MAP custom-config 기본 예제의 완결성을 별도 runtime 개선 작업으로 정리한다.

## 2026-08-05 — MyRePo legacy project README audit

- `Intelligence_capston_opencv/play_video.py`: FPS가 0이거나 유효하지 않은 경우와 frame decode/write 실패를 검사한다.
- `Pattern Recognition`: 손상됐거나 현재 model 구조와 호환되지 않는 checkpoint를 읽을 때 복구 방법이 포함된 진단을 제공한다.
- `ViT_AutoEncoder`: 최소 training batch, 비어 있지 않은 normal/defect 평가 class, 숫자 기준 최신 checkpoint 선택을 검증하고 손상된 GAN checkpoint에 안전하게 대응한다.
- `hangle_normalizer`: 비어 있거나 공백뿐인 reference와 input/output 줄 수를 검증하고 Ollama 요청에 timeout, retry, partial-output 처리를 추가한다.
- `project_bank`: 금액이 숫자·finite·양수인지와 overdraft를 검증하고 실패한 transaction을 내역에 추가하지 않는다. 운영 확장 시 인증·session·CSRF·database 보안을 별도 설계한다.
- `ros2_image_subscribe`: camera 수신 timeout, JPEG decode/write 실패, calibration-aware remap cache, 누락된 frame group 건너뛰기와 호환되지 않는 ONNX model fallback을 처리한다.
