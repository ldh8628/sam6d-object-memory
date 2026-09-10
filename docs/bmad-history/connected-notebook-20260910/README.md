# 연결 노트북 ORB-SLAM3 BMAD 자료 보완

2026-09-10 ORB 소스 커밋 `f2f75950e5bafe5dba402fe2ff797860f0352e48`에 빠졌던 개발 자료를 보존한다. 원본 78개 파일을 출처별로 복사하고 SHA-256을 대조했다.

| 보존 경로 | 원본과 내용 |
|---|---|
| `CLI_environment/_bmad/` | 노트북 `/home/jucpark/DeepLearning/CLI_environment/_bmad/`: SLAM 프로젝트 BMAD 설정·모듈 목록·보조 스크립트 |
| `CLI_environment/_bmad-output/` | 같은 프로젝트의 ORB-SLAM3 D455f IMU 연구 |
| `CLI_environment/_bmad_output_for_slam/` | BMAD 설정이 지정한 실제 산출물: SLAM·SAM 융합 기획, 구현 명세, 연구, 검증 보고서와 그림 |
| `sam6d_object_memory/implementation-artifacts/` | 현재 실행 노트북의 두 호스트·지도 생성·카메라 역할 명세 전체 |
| `sam6d_object_memory/integration/` | 노트북의 GPU SLAM·IMU 실험·분리 실행 안내 |
| `local-controller/implementation-artifacts/` | 제어 호스트 `/home/etri/sam6d_object_memory/`의 ORB CUDA·뷰어, 직접 DDS, 두 호스트 실행 검증 명세 3개 |

노트북의 `CLI_environment/orbslam_ws/_bmad-output/` 자체에는 파일이 없었다. 상위 `_bmad/bmm/config.yaml`은 `project_name: slam_pj`와 `_bmad_output_for_slam` 산출물 경로를 명시한다. 이 경로를 기준으로 자료를 수집했다. 기존 `docs/bmad-history/`의 다른 출처 기록은 유지한다.

이 폴더는 원본 기록의 보존본이다. 과거 절대경로·검증 결과·실행 지침을 현재 코드의 보장으로 해석하지 않는다. Python 캐시만 제외했으며 파일별 원본 경로와 해시는 [manifest.json](manifest.json)에 있다. 현재 저장소의 활성 BMAD 설정을 변경하지 않는다.

저장소 루트에서 무결성 확인:

```bash
sha256sum --check docs/bmad-history/connected-notebook-20260910/SHA256SUMS
```
