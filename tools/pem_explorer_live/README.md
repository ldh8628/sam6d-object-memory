# PEM Explorer v2

드롭 없는 시각화 자료는 trusted SQLite bag을 순차 복원하는 offline 명령으로 만든다.

```bash
python3 tools/capture_pem_explorer.py \
  --config realtime/run_longcircle2_sam_pem_explorer_full.yaml
```

이 명령은 기존 결과를 덮어쓰지 않고 `output/longcircle2_visualization`이 이미 있으면
실패한다. 실시간 처리량 계측은 `realtime/run_longcircle2_sam_pem_explorer.yaml`을
사용한다. 추론의 `READY`가 생긴 뒤 다른 터미널에서 bag을 재생한다.

```bash
ros2 launch realtime/launch/sam6d_split.launch.py \
  config:=realtime/run_longcircle2_sam_pem_explorer.yaml
ros2 bag play data/longcircle2_sam --clock --rate 1.0
```

완료 후 후보를 클릭할 때 필요한 PEM 모델을 지연 적재하는 localhost 서버를 실행한다.

```bash
python3 tools/serve_pem_explorer.py --root output --host 127.0.0.1 --port 8765
```

`--no-model`은 run 분류, 전체 프레임·저장 후보 탐색, 기존 static report만 제공하고
후보 근거 재계산은 비활성화한다. v2는 후보별 이미지나 feature 파일을 생성하지 않는다.
exhaustive run은 수집 중 만든 단일 All-I `preview.mp4`를 우선 사용하고, preview가 없는
run은 source bag JPEG로 복원한다.
`file://`로 이 폴더의 `index.html`을
열면 동적 분석 대신 localhost 실행 안내만 표시한다. 기존 static legacy report는 서버의
landing dropdown에서 그대로 열 수 있다.
