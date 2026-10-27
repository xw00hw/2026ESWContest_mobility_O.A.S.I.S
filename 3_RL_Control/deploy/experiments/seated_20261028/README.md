[← Raspberry Pi 런타임](../../README.md) &nbsp;|&nbsp; ① 센싱 → ② 처리 → **③ RL 제어** → ④ 시트 구동

# 실물 착석 시험 실행 패키지

> 사람이 실제로 앉은 상태에서 전체 시스템(주행 시뮬레이터 → Pi → STM32 → 볼스터)을 구동한 **최종 실물 시험의 실행 코드**입니다.
> 시험 조건을 고정하고 확인하는 래퍼, 카메라 중립 수집기, Pi 화면 표시창, PC 그래프 실행기로 구성됩니다.

---

## 시험 조건

| 항목 | 값 |
|---|---|
| 서보 · 기구 | DS5180 270° 4개, 역기구학(C4_R8)으로 패드 각도 → PWM 변환 |
| 중립 PWM | 우상(D3) 1800 · 좌상(D4) 1800 · 우하(D5) 1200 · 좌하(D6) 1200 µs |
| 각도 범위 | Yaw 0 ~ 20°, Pitch 0°, STM Yaw 속도 10 °/s |
| 시간 | ARM부터 최대 300초 (중립 안정화 2초 + 압력 기준선 2초 포함) |
| 제어 법칙 | G 기본 곡선(0.01 g → 0°, 0.0625 g → 10°, 1 g → 20°) + RL 보정 ±1.5° |
| 예측만 있을 때 | 최대 10°, 100 ms 확인 후 적용 |
| 필터 | yaw 목표에 τ = 0.3 s 저역통과 1회, 속도 1 m/s 미만이면 0° |
| 압력 보호 | 측면 패드 하나라도 원시값 2500 count 이상이면 그쪽 볼스터를 연 상태로 고정 |
| 카메라 | 선택 입력. 양어깨·골반 검출과 작업자가 확인한 중립 보정이 있어야 사용 |

> 1 g에서 20°는 필터 **전** 요청 각도입니다. 속도 제한과 하위 제어 때문에 실제 축이 즉시 20°가 된다는 뜻은 아닙니다.
> 보드가 보고하는 각도는 명령 상태이며, 압력 count는 보정된 접촉력이 아닙니다.

## 구성

| 파일 | 역할 |
|---|---|
| `occupied_trial.py` | **시험 실행 래퍼.** 영점·튜닝 revision·각도 상한이 시험 조건과 정확히 같은지 확인하고, 다르면 ARM을 거부 |
| `raw_count_guard.py` | 압력 원시값 2500 count 보호 |
| `startup_alignment.py` | 시작 시 중립 정렬·안정화 확인 |
| `prepare_candidate.py`, `candidate.yaml`, `package_template.py` | 실행 설정 생성 (장치 경로·확인 항목은 비워 둔 상태로 시작) |
| `camera/collect_pose_reference.py` | 카메라 중립 자세 수집 (작업자 확인 필요) |
| `native/rtmpose_viewer.py` | Pi 화면에 RTMPose 결과 표시 (GTK) |
| `pc/start_pc_displays.py` | PC 그래프 모니터 실행 (SSH, 읽기 전용) |
| [`model_recipe/`](model_recipe/README.md) | RTMPose를 Hailo + CPU로 분할하는 변환·검증 코드 |
| `source_provenance.json` | 실제 시험에 쓴 원본 파일의 SHA256 |
| `test_*.py` | 장치 없는 단위 시험 |

## 실행 방법

### 1. 설정 만들기 (장치를 열지 않음)
```bash
PKG="$REPO/3_RL_Control/deploy/experiments/seated_20261028"
python "$PKG/prepare_candidate.py" --source "$PKG/candidate.yaml" \
  --output /absolute/new_candidate.yaml --expected-tuning-revision 2
```
생성된 파일에 모델·패드 보정·STM 포트·허용 PC 주소·로그 경로를 실제 설치에 맞게 넣습니다.
확인 항목(`confirmations`, 시험 승인 등)은 **그 시점의 장치 상태를 사람이 확인한 뒤**에만 true로 바꿉니다.

### 2. 시험 실행
```bash
python "$PKG/occupied_trial.py" \
  --runtime-root "$REPO/3_RL_Control/deploy/runtime" \
  --config /absolute/reviewed_config.yaml --stop-file /absolute/unique_trial.STOP
# 정지: 지정한 STOP 파일을 만듭니다. 자동 재ARM은 없습니다.
```

### 3. 화면 표시 (선택)
```bash
python3 "$PKG/native/rtmpose_viewer.py" --url http://127.0.0.1:8081           # Pi: 카메라 자세 화면
```
```powershell
python pc/start_pc_displays.py --config C:/absolute/pc_display_config.json --start   # PC: 그래프 (http://127.0.0.1:8766/)
```
카메라 프레임이 끊기면 Pi 화면은 마지막 영상을 유지하면서 `WAITING`과 경과 시간을 표시합니다. 화면에 남은 영상은 제어 입력으로 쓰이지 않습니다.

## 검증 (장치 없이)

```bash
export OASIS_TRIAL_TEST_RUNTIME="$REPO/3_RL_Control/deploy/runtime" PYTHONPATH="$OASIS_TRIAL_TEST_RUNTIME"
python -m unittest discover -s . -p 'test_*.py'
python -m unittest discover -s camera -p 'test_*.py'
python -m unittest discover -s native -p 'test_*.py'
python -m unittest discover -s pc -p 'test_*.py'
```

실제 모델이 필요한 시험은 `OASIS_TEST_MODEL_DIR`이 없으면 건너뜁니다.

## 시험 결과 요약

| 항목 | 값 |
|---|---|
| 제어 구간 | 271초, RL 계산 5,421회, 보드 명령 2,766회 |
| 횡가속도 | −1.21 g ~ +1.26 g, \|G\| ≥ 1인 78회 모두 요청각 20° |
| 최대 보드 명령 | 좌 18.9° · 우 19.9° |
| 압력 | 12채널 스캔 27,777회 정상, 2500 count 보호 발동 0회 |
| 종료 | 모든 세션에서 PWM OFF 확인 |
