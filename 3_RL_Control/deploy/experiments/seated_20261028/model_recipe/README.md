[← 실물 착석 시험 실행 패키지](../README.md) &nbsp;|&nbsp; ① 센싱 → ② 처리 → **③ RL 제어** → ④ 시트 구동

# RTMPose Hailo 분할 추론 변환 레시피

> 운전자 자세 추정 모델(RTMPose)을 **Hailo-10H NPU + CPU**로 나눠 실행하기 위해 사용한 변환·검증 코드입니다.

---

## 왜 나눴나

RTMPose 전체를 Hailo 컴파일러(DFC 5.4)로 변환하면 두 군데에서 막혔습니다.

1. 출력부(SimCC head)의 MatMul 변환이 지원되지 않았습니다.
2. 마지막 7×7 합성곱(`Conv_205`)은 Hailo 자원 배치에 실패했습니다.

그래서 그 직전인 `Mul_204` 지점에서 모델을 둘로 나눴습니다.

```text
카메라 프레임 ─► [Hailo-10H] RTMPose 앞부분 (~ Mul_204) ─► [CPU] 7×7 합성곱 + SimCC 출력부 ─► 관절 좌표
```

분할한 부동소수점 모델은 50개 입력에서 원본과 일치했고, Pi에서 CPU 뒷부분만 실행한 32개 입력 비교도 통과했습니다(최대 SimCC 차이 1.42e-6, 중앙값 4.42 ms).

## 구성

| 경로 | 내용 |
|---|---|
| `model_transform/split_before_final_conv.py` | 원본 RTMPose를 `Mul_204` 경계에서 앞·뒤 두 모델로 분리 |
| `model_transform/validate_hybrid_hardware.py` | 모델·배열·해시 검증 (기본). `--run-device`를 주면 Hailo 장치에서 실제 추론 |
| `dfc54/` | Hailo DFC 5.4 작업 환경에서 쓴 검증·컴파일·매핑 추출·manifest 준비 스크립트 |

## 사용 시 주의

- 이 폴더는 **변환 과정의 기록**이며 바로 실행되는 설치기가 아닙니다. 스크립트에는 당시 작업 환경의 경로와 모델 경계·해시 조건이 고정되어 있습니다.
- `dfc54/`의 일부 파일은 import만 해도 변환을 시작하므로 라이브러리처럼 import하지 않습니다.
- 모델 가중치, ONNX/HAR/HEF 파일, 보정 데이터, Hailo 설치 파일은 포함하지 않습니다.
- 수치 일치는 원본 모델과의 비교이며, 사람 자세 각도의 정확도를 검증한 것은 아닙니다.
- 실제 사용하는 런타임 코드는 [`deploy/runtime/oasis_rt/rtmpose_hailo.py`](../../../runtime/oasis_rt/rtmpose_hailo.py)입니다.
