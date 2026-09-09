# 단일 노트북 BMAD 자료 대조 기록

2026-09-09 기준으로 로컬 BMAD 자료를 `notebook`의 `348e0e911759`와 대조했다.
노트북 한 대에서 SAM-6D, ORB-SLAM3, ObjectMemory를 실행하는 개발 계통의 명세·설정·스킬·연구·검증 근거를 보존한다. 카메라 두 대는 컴퓨터 두 대와 구분한다.

총 6,831개 원본 파일: **추가 5,855개 / 기존 동일 54개 / 제외 922개**.
포함 파일 중 내용이 다른 경로와 동일한 항목은 3,788개다. 출처별 경로를 유지해 서로 다른 시기의 동명 문서와 설치 구성을 덮어쓰지 않았다. Git은 같은 내용의 blob을 공유한다.

## 지금 사용할 문서

- [통합 실행 안내](../../integration/README.md): 단일 호스트 카메라·지도·실시간 실행.
- [현재 통합 명세](../../_bmad-output/implementation-artifacts/): 지도/URDF, CPU 스레드, 변환, 뷰어.
- [현재 SAM-6D 명세](../../sam6d_ws/implementation-artifacts/): ISM/PEM, Explorer, 녹화, 자세 동기화.
- [현재 연구 자료](../../planning-artifacts/research/).
- [현재 BMAD 설정](../../sam6d_ws/_bmad/): 기존 2개 파일을 유지했다.

## 보존한 출처

| 로컬 원본 | 추가 | 기존 동일 | 제외 |
|---|---:|---:|---:|
| `/home/etri` | 220 | 0 | 1 |
| `/home/etri/CLI_environment` | 1 | 0 | 0 |
| `/home/etri/CLI_environment/li_init` | 0 | 0 | 912 |
| `/home/etri/CLI_environment/orbslam_ws` | 1048 | 0 | 0 |
| `/home/etri/CLI_environment/sam6d_jitter_ws` | 984 | 0 | 0 |
| `/home/etri/CLI_environment/sam6d_ws` | 1654 | 0 | 0 |
| `/home/etri/CLI_environment/sam_orb_ws` | 1567 | 0 | 0 |
| `/home/etri/sam6d_object_memory` | 3 | 54 | 9 |
| `/home/etri/slam-perception-stack` | 378 | 0 | 0 |

- [CLI_environment](CLI_environment/): SAM-6D 초기 FP/jitter 조사·PRD, SLAM 융합 실험, 관련 프로젝트의 `_bmad`와 `.agents/skills` 설치본. SAM/ORB 프로젝트에 설치된 BMAD 확장 모듈도 설치 상태의 일부로 보존한다. 설치되어 있다는 사실이 프로젝트에서 모든 워크플로를 사용했다는 뜻은 아니다.
- [slam-perception-stack](slam-perception-stack/): `_bmad`, `_bmad-output`, `_bmad_output_for_slam`, SAM-6D 내부 산출물, ObjectMemory 재개 계획·재구성 안내·스토리 보고서 및 근거 문서.
- [현재 개발 환경의 공용 BMAD 스킬](home-setup/.agents/skills/).
- [외부 BMAD 모듈 라이선스](licenses/). BMAD Method 라이선스는 `slam-perception-stack/node_modules/bmad-method/LICENSE`에 보존했다.

과거 문서는 **작성 당시의 기록**이다. 옛 환경 이름·절대경로·미구현 제안·검증 결과를 현재 실행 계약으로 바꾸지 않았다. 과거 코드/대용량 원본 bag을 가리키는 링크는 이 자료 묶음만으로 열리지 않을 수 있다. 단일 머신 센서 검증에 연결된 다중 머신 동기화 연구는 배경 자료로 보존하며, 현재 두 호스트 마이그레이션 실행 명세는 제외했다.

`_bmad-output`은 산출물이고 `_bmad`는 설정·모듈·보조 스크립트, `.agents/skills`는 워크플로와 템플릿이다. 여기의 설치본은 출처별 보존본이며 현재 루트의 스킬 설정을 자동 교체하지 않는다.

## 대조 방법과 제외 범위

[manifest.json](manifest.json)에 원본 경로, 저장 위치, 크기, SHA-256, 포함/제외 사유와 내용이 동일한 다른 경로를 파일마다 기록했다. `already-present`는 브랜치의 기존 파일과 바이트가 같다는 뜻이다. 원본 내용과 실행 권한을 보존했으며 런타임 코드는 수정하지 않았다.

홈 디렉터리에서 `_bmad*`, `planning-artifacts`, `implementation-artifacts`를 탐색하고, 연결된 프로젝트의 `.agents/skills`와 직접 참조 문서를 추가 대조했다. Git/패키지 캐시·빌드·설치 산출물과 원본 bag/output 폴더 전체는 탐색 범위에서 제외했으며, 별도 `input_validation_20260908/implementation-artifacts`도 확인했다. BMAD 산출물 폴더 안의 그림·CSV·JSON·NumPy 배열 등 검증 첨부물은 포함했다.

제외: 현재 두 호스트 마이그레이션 명세·리뷰, 별도 LiDAR-IMU/Fast-LIO 설치 프로젝트, 홈의 Husky USB 설치 연구, Python 실행 캐시. 구체적인 제외 파일은 manifest의 `excluded` 항목으로 확인할 수 있다.

저장소 루트에서 다음 명령으로 포함된 원본 파일 전체의 무결성을 확인할 수 있다.

```bash
sha256sum --check docs/bmad-history/SHA256SUMS
```

원본 문서에 기록된 과거 테스트를 이번 자료 업로드에서 다시 실행한 것은 아니다. 이번 검증은 파일 누락·내용 일치·Git 변경 범위·원격 반영 여부를 대상으로 한다.
