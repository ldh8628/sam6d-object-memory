# PEM Explorer v2

연구 실행은 `realtime/run_longcircle2_sam_pem_explorer.yaml`을 사용한다. 추론의
`READY`가 생긴 뒤 다른 터미널에서 `data/longcircle2_sam` bag을 재생한다.

```bash
ros2 launch realtime/launch/sam6d_split.launch.py \
  config:=realtime/run_longcircle2_sam_pem_explorer.yaml
ros2 bag play data/longcircle2_sam --clock --rate 1.0
```

완료 후 PEM 모델을 미리 적재하는 localhost 서버를 실행한다.

```bash
python tools/serve_pem_explorer.py --root output --host 127.0.0.1 --port 8765
```

`--no-model`은 run 분류와 정적/partial 탐색만 시험한다. v2는 후보별 이미지나 feature
파일을 생성하지 않으며 `explorer_manifest.json`, `explorer_index.jsonl`,
`candidates.bin`, `replay.bin`, 프레임별 bitset Mask PNG만 읽는다. `file://`로 이 폴더의
`index.html`을 열면 동적 분석 대신 localhost 실행 안내만 표시한다.
