---
title: '변환 출력 기본 경로'
type: 'feature'
created: '2026-08-31'
status: 'done'
route: 'one-shot'
context: []
---

# 변환 출력 기본 경로

## Intent

**Problem:** `convert_all.py` 실행마다 입력 이름을 반복해 `--outputs`에 적어야 했다.

**Approach:** `--outputs`를 생략하면 `output/converted/<inputs 폴더명>`을 사용하고, 명시한 경로는 계속 우선한다. 입력과 출력이 같거나 기존 출력의 원본이 다르면 변환 전에 중단한다.

## Suggested Review Order

**기본 경로**

- 입력 폴더명으로 기본 목적지를 만들고 명시값은 그대로 우선한다.
  [`convert_all.py:546`](../../converting_launch/convert_all.py#L546)

**데이터 보호**

- 입력 삭제와 같은 이름의 다른 원본 재사용을 변환 전에 차단한다.
  [`convert_all.py:579`](../../converting_launch/convert_all.py#L579)
