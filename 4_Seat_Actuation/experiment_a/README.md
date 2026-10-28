[← 4. 시트 구동](../README.md) &nbsp;|&nbsp; ① 센싱 → ② 처리 → ③ RL 제어 → **④ 시트 구동**

# 보드 통신 · 차량 입력 검증 · 백업 제어기 (experiment_a)

> 시트 보드를 안전하게 다루기 위한 **공통 호스트 모듈**과, 학습 모델 없이도 동작하는 **차량 데이터 기반 백업 제어기**입니다.
> Raspberry Pi의 RL 런타임(`3_RL_Control/deploy/runtime/oasis_rt/fbstack.py`)이 이 모듈들을 그대로 import합니다.

---

## 최종 시스템에서 쓰이는 모듈

| 모듈 | 역할 | 사용처 |
|---|---|---|
| `transport.py` | **BoardLink** — OASIS_FALLBACK v2 UART 링크. 요청 하나만 대기, 명령별 정확한 ACK, 명시적 세션 ARM | Pi 런타임 |
| `vehicle.py` | **차량 입력 검증** — 품질 플래그, 순번·스트림·세션 연속성, 200 ms 신선도, 수치 무결성 | Pi 런타임 |
| `impact.py` | **충격 의심 판정** — 평면 가속도 크기·급변·지속시간·구간 적분 | Pi 런타임 (모든 모드) |
| `cad_limits.py`, `cad_profile_data.py` | **CAD 간섭 한계** — 펌웨어와 같은 SHA256 프로파일 | Pi 런타임 엔벨로프 |
| `kinematics.py` | 역기구학 C4_R8, 270° / 2000 µs 펄스 변환 | Pi 런타임 |
| `controller.py`, `settings.py`, `tuning.py` | **백업 제어기** (아래) | 모델이 없을 때 Pi 런타임 |
| `session_logging.py`, `pressure_logging.py` | 압력 12채널 원시 기록 (스키마 공용) | Pi 런타임 |
| `steering_preview.py` | 조향 기반 소폭 선제 반응 | 백업 제어기 |
| `desktop.py`, `runner.py`, `collector_runtime.py`, … | PC 단독 실행용 데스크톱 앱 | PC 시험 |

## 백업 제어기의 제어 법칙

학습 모델 없이 **주행 시뮬레이터의 차량 데이터만** 사용합니다. 압력·카메라·체형 정보는 쓰지 않습니다.

```text
횡가속도 ─► 0.15 s 저역통과 ─► 히스테리시스 (0.03 g 켜짐 / 0.02 g 꺼짐)
          ─► 밀릴 쪽 볼스터 Yaw  = 설정 상한 × clip((|a_y| − 0.02 g) / (0.30 g − 0.02 g), 0, 1)
          ─► Pitch              = atan2(a_y, g)를 패드 면에 대응, CAD 한계 적용
          ─► 각 축 2 °/s 변화율 제한 ─► 반대쪽으로 바뀔 때는 중립을 거침
```

- 좌회전(a_y > 0)이면 몸이 오른쪽으로 밀리므로 **오른쪽** 볼스터를 닫습니다.
- 기본 상한은 Yaw 15°, Pitch ±6°입니다. 큰 횡가속도는 상한에 **포화**시키고 멈추지 않습니다.
- **조향 선제 반응:** 속도 5 m/s 이상에서 핸들을 5° 넘게 0.1초 유지하면, 횡가속도가 생기기 전에 Yaw 최대 2°까지 미리 움직입니다(한 번에 최대 1.5초). 실제 횡가속도 반응이 시작되면 그쪽이 우선합니다.

시뮬레이션 평가에서 이 제어기는 볼스터 고정보다 낫지만 RL 정책·튜닝 규칙보다는 약합니다(J 1.70 vs 3.92). 그래서 **최후의 백업**으로만 사용합니다.

## 충격 의심 판정

다음 조건을 **모두** 만족하면 충격 의심으로 보고 모터를 끕니다.

| 조건 | 기준 |
|---|---|
| 평면 가속도 크기 `√(a_x² + a_y²)` | ≥ 25 m/s² |
| 고가속도 지속 | ≥ 0.06 s |
| 구간 내 급변 (jerk) | ≥ 150 m/s³ |
| 고가속도 구간 벡터 적분 | ≥ 1.5 m/s |

또는 평면 가속도가 60 m/s² 이상이면 지속 시간을 기다리지 않고 바로 끕니다.
이는 시험용 휴리스틱이며, 검증된 충돌 분류기는 아닙니다.

## CAD에서 산출한 각도 한계

실제 3D 모델(STEP)에서 간섭과 링크 전달각을 검사해 구조물 Yaw별 Pitch 허용 범위를 정했습니다.

| 구조물 Yaw | 0° | 6° | 15° | 18° | 20° | 21° | 22° |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Pitch 절대 상한 | 0° | ±6° | ±6° | ±3.6° | ±2° | ±1° | 0° |

- 사이 값은 선형 보간합니다. 22°는 기하학적 절대 한계입니다.
- 검사 범위: 중립 208개, 가동 그룹 간 간섭 51자세, 주요 부품 간격 437자세, 운동학 45,141자세. 최소 간격 0.714 mm, 최소 전달각 사인 0.781.
- 같은 프로파일을 PC/Pi와 펌웨어가 SHA256으로 대조해, 서로 다른 한계를 쓰는 일을 막습니다.

## 실행 방법

```powershell
# 4_Seat_Actuation 폴더에서
python -m experiment_a --config experiment_a/config.candidate.json inspect                     # 설정 확인 (장치 열지 않음)
python -m experiment_a --config experiment_a/config.candidate.json replay --vehicle-csv <vehicle.csv> --out <새 폴더> --plot   # 기록 재생
python -m unittest discover -s experiment_a/tests -q
```

PC 데스크톱 앱 설치·사용은 [`DESKTOP.md`](DESKTOP.md), 입력 경계·시작 권한·종료 규칙 상세는 [`RUNTIME.md`](RUNTIME.md), 몸통 응답 모델과 패드 면 정렬은 [`HUMAN_MODEL.md`](HUMAN_MODEL.md), 시험·재생·에뮬레이션 결과는 [`VALIDATION.md`](VALIDATION.md)를 참고합니다.

> 데스크톱 앱에서 `중지`는 모터 OFF와 기록 종료만 하고, 기록은 사용자가 `저장`이나 `버리기`를 고를 때만 확정합니다. 오류 후 자동으로 다시 ARM하지 않습니다.
