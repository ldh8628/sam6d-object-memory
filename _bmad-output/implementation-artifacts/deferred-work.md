# Deferred work

- `sam6d_ws/tests/test_pem_explorer.py`의 2개 계약 테스트가 과거 생성물 `output/pem_explorer/index.html`과 `output/longcircle2_pem_explorer/README.md` 부재로 실패한다. 이번 260826 ObjectMemory 변경과 무관하므로 해당 Explorer 생성 워크플로를 재현할 때 복구한다.
- `integration/camera_extrinsic_localization.py`의 기존 결과 재사용 검사는 trajectory 행 수와 Atlas 파일 존재만 확인해, 잘렸거나 다른 실행에서 온 Atlas의 무결성까지 증명하지 못한다. ORB-SLAM3 Atlas에 안전한 read-only 검증 방법을 마련할 때 강화한다.
