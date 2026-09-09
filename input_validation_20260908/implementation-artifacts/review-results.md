# 검토 반영

독립 edge-case/acceptance 검토와 추가 재확인을 수행했다. 다음 항목은 기존 계획으로 결정 가능한 구현 결함으로 분류하고 수정했다.

| 발견 | 수정·검증 |
|---|---|
| replay publisher 시작 전에 ORB READY가 막힘 | replay는 publisher 없이 Atlas READY 허용, 이후 QoS 검사는 유지; 실제 ORB 검사 통과 |
| shutdown 직전 DDS 미수신 tail 소실 | ORB/SAM drain service에서 DDS take 후 단일 worker drain; SAM 8개 처리+2개 DDS 대기→10/10 |
| overflow 후 오류 항목이 무한 증가 | 첫 overflow에 intake 중단, 기존 큐 drain, 이후 거부는 카운터만 집계 |
| depth stamp가 RGB보다 빠르면 정상 입력 FAIL | paired depth 구간으로 독립 metadata 창 선택; ±offset 검사 |
| 초기 host stall을 watermark가 숨겨 PASS | 고정 host 창 FPS와 양쪽 경계 간격 검사 |
| stamp와 counter 동시 reset을 INCOMPLETE로만 판정 | host 창의 reset 사건도 FAIL 검사 |
| 다른 serial의 consumer 기록을 허용 | 실제 로그 serial과 설정을 대조 |
| 부하 실행을 camera-only baseline으로 허용 | baseline 역할·부하·profile·RMW·sync·publisher XML 증거 검사 |
| 잘린 JSONL에서 최종 요약 미생성 | 읽을 수 있는 증거 보존, 오류를 INCOMPLETE에 기록 |
| CDR alignment padding으로 허위 내용 불일치 | 모든 ROS 의미 필드와 정확한 image buffer로 비교 |
| SQLite tail 삭제 검사가 수량 확인에 그침 | 실제 변환 검증 실패를 assertion; native와 SQLite 실제 ROS replay 추가 |
| 고정 map 이름을 활성 Atlas ID처럼 사용 | 잠금된 core GetCurrentMapId, map 변경 발행 및 프레임별 실제 Atlas ID 기록 |

마지막 acceptance 재확인에서 명백한 잘못된 PASS나 새 종료 데이터 손실은 발견되지 않았다. 이 검토는 실제 두 카메라 성능 합격을 대신하지 않는다.

남은 항목은 [검증 보고서](../verification/REPORT.md)의 하드웨어 미달·미실행 결과다. Legacy ApproximateTime의 서로 다른 stamp tail 보류는 기존 호환 경로의 제한이며 native 무손실 주장에 포함하지 않는다.

## 두 카메라 연결 후 추가 검토 — 2026-09-08

- 검사 종료 watermark 직전 프레임을 metadata/RGBD에서 동시에 제거하면 PASS가 될 수 있는 경계를 수정했다. RGB/depth 독립 offset의 첫/마지막 프레임 누락 회귀 검사를 추가했고 기존 하드웨어 보고서 네 건을 원본 로그로 재판정하여 PASS를 확인했다.
- ORB가 최초 READY 뒤에도 publisher QoS 변경을 검사한다. reliable publisher를 best-effort로 교체하는 실제 ROS 회귀 검사 통과.
- 두 카메라 종료를 병행하고 부모 wrapper 종료 예산을 자식보다 길게 보장한다. 두 자식 모두 SIGINT를 받아야 종료하는 실제 프로세스 검사로 검증했다.
- 기본 실행의 baseline 누락 INCOMPLETE 회귀를 수정했다. 이미 실행된 카메라로 30초 기준 측정을 자동 수행하며 실패 시 부하를 시작하지 않는다. 실제 두 카메라 검증 통과.
- 소프트웨어 검사 15개, ORB 10개 입력 모드 및 core CTest, SAM DDS tail 10/10, 실제 ROS MCAP/SQLite 검사를 실제 프로젝트 적용 경로에서도 통과했다.
- 선택 ORB map viewer는 불량 장면의 반복 reset 중 SIGSEGV가 발생하여 별도 조사 중이다. 기본 지도 촬영 경로는 30초 899/899쌍(각 카메라), 녹화·SQLite 변환 모두 PASS했다.
