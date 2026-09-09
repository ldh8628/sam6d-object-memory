---
stepsCompleted: [1, 2, 3, 4]
inputDocuments: []
workflowType: 'research'
lastStep: 1
research_type: 'technical'
research_topic: '조명 불변 실시간 색감 전처리'
research_goals: '밝기 절댓값을 사용하지 않고 현재 ISM에 결합 가능한 빠른 색감 정규화 방법을 조사하고 실제 milk/GPU 상자 데이터로 검증한다.'
user_name: 'Etri'
date: '2026-08-29'
web_research_enabled: true
source_verification: true
---

# Research Report: technical

**Date:** 2026-08-29
**Author:** Etri
**Research Type:** technical

---

## Research Overview

[Research overview and methodology will be appended here]

## Technical Research Scope Confirmation

**Research Topic:** 조명 불변 실시간 색감 전처리
**Research Goals:** 밝기 절댓값을 사용하지 않고 현재 ISM에 결합 가능한 빠른 색감 정규화 방법을 조사하고 실제 milk/GPU 상자 데이터로 검증한다.

**Technical Research Scope:**

- Architecture Analysis - 현재 ISM HSV gate와 결합 지점
- Implementation Approaches - 학습 없는 색상 정규화 및 불변량
- Technology Stack - 현재 Python/OpenCV/NumPy 환경에서 재사용 가능한 구현
- Integration Patterns - 기존 mask 기반 histogram 앞단 또는 대체 채널
- Performance Considerations - CPU 프레임당 처리시간과 메모리

**Research Methodology:**

- 최신 공개 자료와 원 논문/공식 구현 검증
- 핵심 주장에 대한 복수 출처 교차 확인
- 실제 milk/GPU 상자 프레임의 분리도 및 밝음/어두움 데이터 비교
- 불확실한 결론에는 신뢰 수준 표기

**Scope Confirmed:** 2026-08-29

## Technology Stack Analysis

### Programming Languages

현재 시스템은 Python에서 마스크·RGB 배열을 처리하고 있으므로 별도 언어나 런타임을 추가할 이유가 없다. 후보 변환은 모두 픽셀별 사칙연산, histogram, 선택적 blur/derivative로 표현할 수 있어 NumPy 벡터화로 충분하며, 최종 운영 경로가 C++로 이동해도 동일 수식을 그대로 옮길 수 있다.

_권장 언어:_ 기존 Python/NumPy 유지  
_성능 특성:_ normalized-rgb·log-chromaticity는 O(N) 단일 패스, Gray-World/Shades-of-Gray도 O(N), Gray-Edge는 derivative 때문에 추가 메모리 접근이 필요하다.  
_신뢰도:_ 높음

### Development Frameworks and Libraries

OpenCV는 HSV, Lab, Luv, YCrCb 등 필요한 색공간 변환을 이미 제공한다. 다만 이번 목적은 단순 색공간 변경이 아니라 조명 세기 또는 광원 색을 제거하는 것이므로 `cvtColor`만으로는 부족하다. OpenCV `xphoto`에는 Gray-World white balance가 있지만, 장면 평균이 무채색이라는 가정에 의존한다. [OpenCV 색공간 변환 공식 문서](https://docs.opencv.org/4.12.0/d8/d01/group__imgproc__color__conversions.html), [OpenCV GrayworldWB 문서](https://docs.opencv.org/4.x/db/d99/classcv_1_1xphoto_1_1GrayworldWB-members.html)

Gevers와 Smeulders는 normalized `rgb`, `c1c2c3`, `l1l2l3`, 이웃 색 비율 계열을 물리적 색상 불변량으로 비교했다. 단순 normalized `rgb`는 조명 **세기**에는 강하지만 광원 스펙트럼이 바뀌면 성능이 저하되고, 이웃 픽셀 색 비율 `m1m2m3`가 광원 색 변화에 더 강하다는 결과다. [원 논문 PDF](https://staff.fnwi.uva.nl/th.gevers/pub/GeversPR99.pdf)

_최소 라이브러리:_ 현재 NumPy; OpenCV는 기존 의존성이므로 재사용 가능  
_추가 의존성:_ 없음  
_신뢰도:_ 높음

### Database and Storage Technologies

학습 데이터베이스나 신규 feature store는 필요하지 않다. 현재 42-view template의 정규화 색 histogram만 기존 HSV prototype처럼 한 번 계산해 캐시에 저장하면 된다. 캐시 형식도 현행 NumPy `.npz`를 재사용할 수 있다.

_권장 저장:_ 기존 template별 `.npz`  
_신규 DB:_ 불필요

### Development Tools and Platforms

검증은 현재 저장된 `frames.jsonl`, ISM mask, preview/source RGB를 읽어 실제 milk와 GPU 상자 점수 분포를 재계산하는 작은 오프라인 스크립트면 충분하다. 신규 테스트 프레임워크는 필요 없고, 현재 테스트 체계에 threshold 경계와 해당 회귀 프레임만 추가하는 구성이 적절하다.

White-balance 계열 중 Shades-of-Gray는 Gray-World와 Max-RGB를 Minkowski norm으로 일반화한다. Gray-Edge는 장면 평균 대신 영상 derivative 평균이 무채색이라는 가정을 사용하며 계산 효율이 장점으로 보고됐다. 그러나 둘 다 장면 전체에서 광원을 추정하므로 좁은 객체 crop만 입력하면 특정 물체색이 추정치를 편향시킬 수 있다. [Shades-of-Gray 원 논문](https://library.imaging.org/admin/apis/public/api/ist/website/downloadArticle/cic/12/1/art00008), [Gray-Edge 원 논문](https://staff.science.uva.nl/th.gevers/pub/GeversTIP07.pdf)

### Cloud Infrastructure and Deployment

클라우드, 컨테이너, 외부 서비스는 적용 대상이 아니다. 전처리는 카메라 프레임과 mask가 있는 로컬 ROS/SAM-6D 프로세스 내부에서 수행해야 지연과 데이터 복사를 최소화한다.

_배포:_ 기존 로컬 GPU/CPU 프로세스  
_외부 서비스:_ 불필요

### Technology Adoption Trends

후보 기술은 세 부류다.

1. **픽셀 색 불변량:** normalized `rg`, opponent/log-chromaticity, `c1c2c3` — 가장 빠르지만 광원 색 변화에는 한계가 있다.
2. **장면 기반 color constancy:** Gray-World, Shades-of-Gray, Gray-Edge — 저비용이지만 장면 통계 가정이 필요하다.
3. **intrinsic/Retinex:** 조명·그림자 제거 능력은 더 크지만 여러 scale의 convolution 또는 영상별 최적화가 필요해 현재 실시간 gate에는 과하다. [Entropy-minimization intrinsic image 연구](https://www.cs.sfu.ca/~mark/ftp/Eccv04/), [Multiscale Retinex 분석 및 구현](https://www.ipol.im/pub/art/2014/107/article.pdf)

현재 문제에 대한 1차 우선순위는 **장면 전체에 간단한 Gray-World/Gray-Edge 보정 후, mask 내부에서 normalized chromaticity 또는 이웃 색 비율 histogram을 비교**하는 것이다. 이 조합은 절대 밝기를 사용하지 않고, 새 모델이나 의존성을 추가하지 않는다.

_신뢰도:_ 중간-높음. 방법론은 검증됐지만 이 카메라·milk template에 대한 실제 분리도는 다음 단계의 데이터 실험이 필요하다.

## Integration Patterns Analysis

### API Design Patterns

외부 REST/gRPC API는 필요하지 않다. 가장 작은 결합점은 현행 `ism_hsv.shadow_score(bgr, box, mask, proto)`의 내부 표현을 교체하거나 병렬 score를 추가하는 것이다. 현재 단일 객체 경로와 multi-label 경로가 모두 이 색상 모듈을 호출하므로, 호출부마다 gate를 추가하는 것보다 공통 함수 한 곳을 바꾸는 편이 동작 불일치를 막는다.

권장 함수 계약은 기존과 동일하다.

```text
color_score(bgr_frame, bbox, mask, template_proto) -> [0, 1]
```

프레임 전체가 필요한 illuminant 추정치는 frame stamp별로 한 번만 계산하고, 후보별 함수에는 준비된 통계값을 넘기거나 프레임 객체에 캐시한다. normalized chromaticity 및 histogram은 실제 ISM 후보 mask 안에서만 계산한다.

### Communication Protocols

ROS topic이나 메시지를 추가할 필요가 없다. 원본 RGB는 이미 `sensor_msgs/Image`로 들어오며, 이 메시지의 header stamp는 촬영 시각이고 encoding이 픽셀 채널 의미를 정의한다. 기존 BGR 변환 직후 동일 프레임 배열을 재사용하면 복사와 동기화 문제가 생기지 않는다. [ROS 2 sensor_msgs/Image 공식 정의](https://docs.ros.org/en/ros2_packages/jazzy/api/sensor_msgs/msg/Image.html)

색상 gate는 ISM 내부 수락 여부만 바꾸고, 통과한 결과는 지금처럼 `vision_msgs/Detection3DArray`로 PEM/ObjectMemory에 전달한다. 따라서 downstream 프로토콜은 그대로 유지된다. [ROS vision_msgs 공식 문서](https://docs.ros.org/en/ros2_packages/rolling/api/vision_msgs/)

### Data Formats and Standards

template reference는 현행 42-view RGB/mask에서 같은 전처리를 수행해 histogram bank로 만든 뒤 기존 `.npz` 캐시 구조를 재사용한다. 운영 JSONL에는 다음 진단값만 추가하면 충분하다.

```json
{"color_invariant_score": 0.0, "color_invariant_pass": false}
```

기존 `hsv_score`를 즉시 덮어쓰면 이전 결과와 의미가 섞이므로, 검증 기간에는 두 점수를 함께 기록하고 최종 전환 후 구 필드를 제거하는 편이 안전하다.

### System Interoperability Approaches

통합 순서는 다음이 적절하다.

```text
RGB frame
  └─ 프레임당 1회: 선택적 illuminant 추정
       └─ 기존 YOLO/DINO/SAM mask
            └─ 후보당 1회: mask 내부 조명 불변 색상 histogram
                 ├─ reject → PEM 호출 안 함
                 └─ accept → 기존 PEM 및 Detection3DArray
```

색상 gate는 PEM 뒤가 아니라 ISM 마지막에 둬야 오검출의 고비용 PEM 실행과 ObjectMemory 유입을 동시에 막는다. 색상 불변량은 밝기 세기에는 강하지만 광원 스펙트럼 변화에 대한 성질이 서로 다르므로, template과 query에 반드시 동일한 변환을 적용해야 한다. [Gevers–Smeulders 색상 불변량 원 논문](https://staff.fnwi.uva.nl/th.gevers/pub/GeversPR99.pdf)

### Microservices Integration Patterns

마이크로서비스, API gateway, service discovery는 적용하지 않는다. 전처리를 별도 ROS node로 분리하면 RGB serialization과 추가 queue latency만 생기며, 하나의 scalar gate를 위해 운영 지점을 늘리게 된다. 현행 ISM 프로세스 내부 함수가 적절하다.

### Event-Driven Integration

새 event bus는 필요 없다. 기존 frame callback에서 후보가 생성된 경우에만 lazy evaluation하고, 결과는 현재 진단 JSONL에 함께 기록한다. 프레임마다 객체가 없으면 색상 histogram 계산 자체를 생략할 수 있다.

### Integration Security Patterns

외부 입력·네트워크·자격 증명이 추가되지 않으므로 별도 보안 경계는 없다. 다만 입력 mask가 비어 있거나 RGB 합이 0에 가까운 픽셀에서 0 나눗셈이 발생하지 않도록 epsilon과 유효 픽셀 하한은 필수다. ROS Image의 encoding이 `bgr8`/`rgb8` 중 무엇인지 확인하고 한 번만 표준화해야 채널 뒤바뀜을 방지할 수 있다. ROS는 두 encoding을 별도로 정의한다. [ROS image encoding 공식 목록](https://docs.ros.org/en/ros2_packages/rolling/api/sensor_msgs/generated/program_listing_file_include_sensor_msgs_image_encodings.hpp.html)

### Integration Recommendation

1. `ism_hsv.py` 한 곳에 새 score를 병렬 추가한다.
2. query와 42-view template에 동일한 변환을 적용한다.
3. 초기에는 판정에 개입하지 않는 shadow mode로 전체 데이터 score를 수집한다.
4. 실제 milk 유지율과 GPU/상자 FP 제거율로 threshold를 고정한다.
5. 검증 후 기존 HSV gate를 대체하고 ROS/PEM/ObjectMemory 인터페이스는 변경하지 않는다.

_신뢰도:_ 높음. 통합 위치와 기존 인터페이스 재사용은 현재 코드 흐름 및 공식 ROS 메시지 계약으로 확인됐다.

## Architectural Patterns and Design

### System Architecture Patterns

현재 문제에는 별도 서비스나 새 ROS node가 필요하지 않다. 가장 작은 구조는 기존 ISM 색상 gate 내부의 **2단계 reject-only cascade**다.

```text
ISM 후보 mask
  └─ 1단계: normalized-rg histogram
       ├─ 명확한 불일치 → reject
       ├─ 명확한 일치 → 기존 PEM 흐름
       └─ 경계 점수 → 2단계 local log color-ratio
```

1단계의 `r=R/(R+G+B)`, `g=G/(R+G+B)`는 채널 전체에 동일하게 곱해지는 밝기 배율을 제거한다. 2단계는 이웃 픽셀 사이의 채널별 log-ratio를 사용해 대각 광원 모델에서 광원 색과 음영의 공통 배율까지 더 강하게 상쇄한다. Gevers와 Smeulders의 비교에서도 normalized `rgb`는 조명 세기에 강하고, 이웃 색 비율 `m1m2m3`는 광원 색 변화에 더 강한 것으로 보고됐다. [색상 불변량 원 논문](https://staff.fnwi.uva.nl/th.gevers/pub/GeversPR99.pdf)

### Design Principles and Best Practices

- 색상 점수는 semantic/appearance 실패를 복구하지 않고 오검출을 거부하는 데만 사용한다.
- template과 query에 같은 변환, 채널 순서, epsilon, histogram bin을 적용한다.
- RGB 합이 거의 0이거나 채널이 clipping된 픽셀은 색상 판별 근거가 아니라 수치적으로 불안정한 표본이므로 제외한다.
- 프레임 전체 white balance를 적용한다면 illuminant는 후보 crop이 아니라 전체 프레임에서 한 번만 추정하고 frame stamp로 재사용한다.
- 운영 전에는 기존 HSV와 새 점수를 동시에 기록하는 shadow mode로 threshold를 결정한다.

### Scalability and Performance Patterns

normalized-rg는 mask 픽셀에 대한 O(N) 벡터 연산과 작은 2D histogram 하나로 끝난다. local log-ratio는 경계 후보에만 실행하므로 대부분의 후보는 첫 단계에서 종료된다. template feature는 42-view마다 한 번 미리 계산해 기존 `.npz`에 저장한다. 따라서 신규 모델, GPU kernel, process 간 복사, 상시 프레임 전처리가 없다.

Gray-World, Shades-of-Gray, Gray-Edge는 전역 광원 색을 보정하는 선택지다. Gray-Edge는 영상 derivative의 평균이 무채색이라는 가정을 사용해 넓은 단색 영역의 편향을 줄이지만, 공간적으로 균일한 광원과 충분한 edge 다양성을 필요로 한다. 실제 데이터에서 normalized-rg와 log-ratio만으로 분리가 안 될 때만 추가하는 것이 적절하다. [Gray-Edge 원 논문](https://staff.science.uva.nl/th.gevers/pub/GeversTIP07.pdf), [Computational Color Constancy survey](https://staff.fnwi.uva.nl/th.gevers/pub/GeversTIP11.pdf)

### Integration and Data Patterns

공통 구현 지점은 `ism_hsv.py`의 score 함수다. 단일 객체 경로와 multi-label 경로가 이 모듈을 공유하므로 호출부는 그대로 두고, 내부에서 기존 HSV score와 새 score를 함께 계산한다. 출력은 기존 scalar score 계약을 유지하고 검증용 진단 필드만 추가한다.

```json
{
  "hsv_score": 0.186,
  "color_rg_score": 0.0,
  "color_ratio_score": null,
  "color_invariant_pass": false
}
```

### Deployment and Operations Architecture

배포 단위, ROS topic, message, launch 파일은 변경하지 않는다. threshold는 실제 positive/negative 분포에서 고정하고 물리 환경 변화에 필요한 하나의 조정값만 남긴다. 새 gate가 예외를 내거나 유효 픽셀이 부족하면 초기 shadow mode에서는 기존 결과를 유지하고 진단값을 남긴다.

### Architecture Recommendation

우선 **normalized-rg histogram을 기존 HSV와 병렬 측정**하고, 분포가 겹치는 경계 후보에만 local log color-ratio를 적용한다. Gray-Edge·Retinex·학습 기반 color constancy는 최초 구현에서 제외한다. 이 구조가 밝기 절댓값을 사용하지 않으면서 기존 코드 한 곳, 기존 의존성, 후보당 O(N) 계산으로 목표를 만족하는 최소 설계다.

_신뢰도:_ 중간-높음. 불변성의 이론적 근거와 통합 구조는 확실하지만, 실제 milk/GPU 상자 분리도와 최종 threshold는 데이터 검증이 필요하다.

---

<!-- Content will be appended sequentially through research workflow steps -->
