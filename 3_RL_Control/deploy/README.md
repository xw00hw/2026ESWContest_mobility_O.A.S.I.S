[← 3. RL 제어](../README.md) &nbsp;|&nbsp; ① 센싱 → ② 처리 → **③ RL 제어** → ④ 시트 구동

# Raspberry Pi 실시간 런타임 (deploy)

> Raspberry Pi 5에서 주행 데이터를 받아 **20 Hz로 볼스터 목표 각도를 계산**하고, STM32 보드(펌웨어 OASIS_FALLBACK v2)에 명령을 보냅니다.
> Pi는 PWM을 직접 다루지 않습니다. 역기구학·각도 한계·변화율·500 ms lease는 펌웨어가 집행합니다.

---

## 한 tick에서 일어나는 일

```text
UDP 64257 ─► beamng_rx      릴레이 수신, Pi 시계로 재타임스탬프, 입력 검증, 조향 부호 감시, 충격 판정
          ─► predictor      0.1 ~ 1.0초 뒤 횡가속도 예측
          ─► controller     관측 57개 구성 → 정책 추론 (C / NumPy) → G 곡선 + RL 보정 × 체형 스케일
          ─► safety         0.3 s 저역통과 → 압력 인터락
          ─► envelope       CAD 한계 ∩ STM 각도 상한 ∩ 역기구학 범위로 투영
          ─► fb_link        FB_SET 전송, 건강한 tick 뒤에만 lease 갱신(FB_PING), 압력 스캔 수신
                                                                    ──USB──► STM32
```

| 제어 모드 | 내용 | 언제 |
|---|---|---|
| `rl` (기본) | 튜닝 규칙 + SAC 잔차 정책 | 모델 패키지가 정상일 때 |
| `rule` | 튜닝 규칙만 | 정책 추론 실패 시 자동 전환 (세션 끝까지) |
| `fb_v6` | 백업 제어기 (차량 데이터만 사용) | 모델 패키지가 없을 때 자동 |

---

## 구성

| 경로 | 내용 |
|---|---|
| `runtime/oasis_rt/app.py` | **진입점.** 세션 전체 관리, `--check-vehicle`(입력 확인), `--sil`(장치 없이 전체 세션) |
| `runtime/oasis_rt/beamng_rx.py` | 릴레이 수신, 재타임스탬프, 차량 입력 검증, 조향 부호 감시 |
| `runtime/oasis_rt/predictor.py` | 1초 횡가속도 예측기 (물리 기반 + MLP 잔차) |
| `runtime/oasis_rt/controller.py` | 20 Hz 제어기: 관측 구성, 정책 백엔드, 인터락, fallback |
| `runtime/oasis_rt/actor.py`, `actor_core.c` | 정책 추론 (C 라이브러리 / NumPy) |
| `runtime/oasis_rt/g_residual_control.py` | G 기본 곡선 + RL 보정 (±1.5°) |
| `runtime/oasis_rt/body_scale.py` | 카메라 기반 체형 스케일 |
| `runtime/oasis_rt/safety.py`, `envelope.py` | 압력 인터락, 안전 엔벨로프 투영 |
| `runtime/oasis_rt/fb_link.py`, `fbstack.py` | STM32 보드 링크 (프로토콜은 `4_Seat_Actuation/experiment_a`를 그대로 사용) |
| `runtime/oasis_rt/pose_camera.py`, `rtmpose_hailo.py`, `pose_input.py` | 카메라 → RTMPose(Hailo + CPU) → 자세 입력 |
| `runtime/oasis_rt/wheel_input.py` | 조향 휠 직접 입력 (선택) |
| `runtime/tests/` | 모의 보드 SIL, 제어기·인터락·예측기·카메라·체형 스케일 단위 시험 |
| `runtime/config.yaml` | 실행 설정 |
| `export/`, `tools/pack_model.py` | 학습 결과 → 모델 패키지(정책, 예측기, 관측 계약, manifest + sha256) → Pi 설치 |
| `tools/pad_calib.py` | 압력 패드 저울 보정 ([절차](tools/PAD_CALIB_PROTOCOL.md)) |
| [`experiments/seated_trial/`](experiments/seated_trial/README.md) | 실물 착석 시험 실행 래퍼와 카메라·모니터 도구 |
| [`CONTROL_SPEC.md`](CONTROL_SPEC.md) | **제어 사양 전체** (관측, 예측기, 규칙, RL, 체형, 안전, fallback, 결과) |

---

## 설치 (Raspberry Pi, 한 번)

```bash
# Raspberry Pi OS Bookworm (64-bit)
git clone https://github.com/xw00hw/2026ESWContest_mobility_O.A.S.I.S.git ~/oasis
ln -s "$HOME/oasis/3_RL_Control" ~/rl
python3 -m venv ~/.venv/oasis && ~/.venv/oasis/bin/pip install -r ~/rl/deploy/runtime/requirements.txt
sudo usermod -aG dialout $USER                      # STM32 ST-LINK 시리얼 접근

# 모델 설치 (PC에서 만든 패키지를 복사)
#   PC: python deploy/tools/pack_model.py deploy/model_v8 && scp dist/oasis_model_v8.tar.gz* <pi>:~/
cd ~/rl && ~/.venv/oasis/bin/python deploy/tools/pack_model.py --install ~/oasis_model_v8.tar.gz --into deploy/
cd deploy/runtime && make MODEL=../model_v8         # C 추론 라이브러리 빌드 (-mcpu=cortex-a76)

# 확인
~/.venv/oasis/bin/python -m oasis_rt.actor --model ../model_v8 --backends c numpy   # 지연·오차 비교
~/.venv/oasis/bin/python -m oasis_rt.app --sil 60                                   # 장치 없이 세션 전체
```

모델 패키지가 없거나 손상되면 백업 제어기가 세션을 맡고, 로그에 `COLD FALLBACK`이 남습니다.
PC와 Pi는 **유선**으로 연결하고, `beamng.allow_from`에 PC 주소를 넣어 다른 송신자를 막습니다.

## 보드 준비 (STM32)

1. 모터 전원을 끈 상태에서 OASIS_FALLBACK v2 펌웨어를 올립니다 ([`stm32/FALLBACK_PROTOCOL.md`](../../4_Seat_Actuation/stm32/FALLBACK_PROTOCOL.md)).
2. 세션에 쓸 각도·속도 상한을 설정합니다 ([`EXPERIMENT_A_LIVE_TUNING.md`](../../4_Seat_Actuation/stm32/EXPERIMENT_A_LIVE_TUNING.md)).

| 설정 | 권장 | 이유 |
|---|---|---|
| `yaw_max_mdeg` | 20000 | 정책의 출력 범위가 0 ~ 20° |
| `yaw_rate_mdeg_s` | 10000 ~ 20000 | 학습 조건. 10 °/s 미만이면 런타임이 RL 대신 규칙을 사용 |
| `pitch_max_mdeg` | 기본값 | 최종 정책은 pitch를 쓰지 않음 (0°) |

Pi는 세션 시작 때 보드의 튜닝 값과 CAD 프로파일 해시를 확인하고 고정합니다. 세션 중 값이 바뀌면 멈춥니다.

## 세션 실행

```bash
cd ~/rl/deploy/runtime
~/.venv/oasis/bin/python -m oasis_rt.app --check-vehicle 30   # 주행 데이터 수신·조향 부호 확인 (보드 열지 않음)
# config.yaml의 confirmations 항목을 사람이 확인한 뒤 true로 설정
~/.venv/oasis/bin/python -m oasis_rt.app                      # Ctrl+C = 정지 (PWM OFF 확인 후 종료)
```

```text
1. 모델·정책 준비 → 시리얼·UDP 열기
2. 보드 확인: HELLO → OFF → CAD 해시 → 튜닝 값 → 압력 스캔 시작 → 상태
3. 유효한 주행 데이터 + PWM OFF가 2초 연속 → ARM 한 번 → 중립 확인
4. 중립에서 압력 기준선 2초 측정 (변동이 크면 시작 거부)
5. 제어 루프: 제어 20 Hz, 보드 PING·상태 100 ms, 명령 ≥ 40 ms, 압력 1초
6. 어떤 오류든 → PWM OFF → OFF 확인 → 종료 (자동 재시작 없음)
```

세션 로그는 `~/oasis_logs/app_<시각>_hw_<고유값>/`에 남습니다.

| 파일 | 내용 |
|---|---|
| `manifest.json` | 결과, OFF 확인, 펌웨어 정보, 모델·모듈 해시, 입력 통계 |
| `control.csv` / `control.jsonl` | tick마다 관측 57개, 행동, 명령, 보고 각도, 인터락 힘, G 제어 상세 |
| `pressure_raw.csv`, `pressure_sweeps.csv` | 압력 12채널 원시 ADC와 MCU·호스트 시각 |
| `INCOMPLETE` | 모든 기록이 안전하게 저장된 경우에만 삭제 |

## 체형 스케일 사용하기 (선택)

기준 사용자와 체형이 다른 사람이 앉을 때만 켭니다.

1. 카메라와 좌석 위치를 기준 측정 때와 같게 둡니다(비율이 화면 대비 값이기 때문).
2. 새 착석자의 카메라 중립 자세를 수집합니다 (`experiments/seated_trial/camera/collect_pose_reference.py`).
3. 실행 설정에 추가합니다.
   ```yaml
   g_residual_control: {enabled: true}
   body_scale: {enabled: true, neutral: /absolute/path/this_person_neutral.json}
   ```
4. 시작 로그의 `body scale 0.9xx (shoulder ratio …)`를 확인합니다.

중립 파일이 없거나, 비율이 0.7 ~ 1.4를 벗어나면(구도가 바뀐 것으로 판단) 시작을 거부합니다.

## 카메라 자세 입력 (선택)

`oasis_rt.pose_camera`는 제어기와 분리된 프로세스입니다. 최신 프레임 하나만 처리하고, 결과를 같은 Pi의 localhost UDP로 제어기에 보냅니다.

```bash
python -m oasis_rt.pose_camera --backend hailo-hybrid --hybrid-manifest <manifest.json> \
  --device /dev/v4l/by-id/<camera> --source-id arducam-rtmpose --preview-port 8081
```

- **Hailo + CPU 분할 추론:** RTMPose의 `Mul_204` 앞부분은 Hailo-10H에서, 마지막 7×7 합성곱과 SimCC 출력부는 CPU에서 계산합니다. 전체 모델은 Hailo 컴파일러의 MatMul 변환과 자원 배치 한계로 한 번에 올라가지 않았습니다. 분할 모델은 원본과 수치 일치(최대 SimCC 차이 약 1.4e-6)를 확인했습니다.
- **안전한 기본값:** 카메라가 없거나 끊기거나, 관절 신뢰도가 낮거나, 오래되었거나, 중립이 보정되지 않았으면 자세 입력만 "사용 불가"로 처리하고 차량·압력 경로는 계속 동작합니다. 재연결되면 중립을 다시 확인해야 합니다.
- 기본 설정(`pose.source: none`)에서는 카메라 소켓을 열지 않습니다.

## 실패하면 무엇이 일어나나

| 상황 | 동작 |
|---|---|
| ARM 전 입력 문제 | 2초 검증을 처음부터 다시 (ARM하지 않음) |
| ARM 후 잘못된 행, 200 ms 이상 끊김, 순서 어긋남 | PWM OFF → 종료 |
| 충격 의심 (평면 가속도 ≥ 25 m/s² 지속 등, 또는 ≥ 60 m/s²) | PWM OFF → 종료 |
| 조향 부호 불일치 | PWM OFF → 종료 (`beamng.steer_sign` 반대로 설정) |
| 정책 예외 / NaN | 튜닝 규칙으로 전환 (세션 끝까지) |
| 예측기 NaN | 그 tick만 현재 값 유지 |
| 모델 없음 / 손상 | 백업 제어기 |
| 보드 오류, 재부팅, ACK 200 ms 초과 | 비상 OFF → 종료 |
| Pi 멈춤 | lease 갱신이 끊겨 500 ms 뒤 펌웨어가 PWM OFF |

> PWM OFF는 신호 상태입니다. 하중이 걸린 기구가 풀리거나 그 자리에 있다는 보장은 아닙니다.

## 성능

- 정책 추론(C): PC 중앙값 6.9 µs, tick 전체 1 ms 미만 / 50 ms 주기. 이 크기의 MLP는 Hailo 이득이 없어 CPU에서 실행합니다.
- 실제 응답 속도는 연산이 아니라 **구동기**가 정합니다. 펌웨어는 서보 펄스를 20 ms마다 2 µs까지만 바꾸므로 yaw 0 → 9°에 약 1.5초가 걸립니다. 그래서 정책도 이 구동기 모델로 학습했습니다.
