---
title: 'Recorder SIGINT와 D455F 스트림 안정화'
type: 'bugfix'
created: '2026-08-27'
status: 'done'
route: 'one-shot'
---

# Recorder SIGINT와 D455F 스트림 안정화

## Intent

**Problem:** `sam6d.sh --record`를 Ctrl-C로 종료하면 FFmpeg가 종료 코드 255를 반환하며, D455F raw Depth 640×480 시작은 endpoint watchdog을 간헐적으로 유발한다.

**Approach:** FFmpeg를 새 세션에서 실행해 recorder가 정상 종료를 제어하고, raw Depth는 안정적인 848×480으로 받아 Color 640×480에 정렬한다.

## Suggested Review Order

- FFmpeg만 신호 그룹에서 분리해 기존 graceful close 경로를 그대로 사용한다.
  [`sam6d_live_recorder.py:92`](../realtime/sam6d_live_recorder.py#L92)

- 안정적인 raw Depth를 사용하면서 최종 RGBD 해상도는 640×480으로 유지한다.
  [`realsense.sh:14`](../run/realsense.sh#L14)
