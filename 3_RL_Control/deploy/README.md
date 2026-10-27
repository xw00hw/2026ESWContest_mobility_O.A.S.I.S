# O.A.S.I.S Raspberry Pi 런타임 (RL-001)

> **2026-10-28 실물 시험 프로파일:** 새 영점(1800/1800/1200/1200 µs), G 기본각 + RL 보정, 카메라 선택 입력, 원시 2500 count 해당측 열림, 기준선 변동 경고 설정은 [날짜별 실행 패키지](experiments/seated_20261028/README.md)를 따른다. generic `oasis_rt.app`의 기본 config는 이 프로파일을 자동으로 활성화하지 않는다. 당일 실제 시험 결과 요약은 [저장소 README](../../README.md)를 참조한다. 아래의 "준비/미배포" 표기는 이전 단계의 기록이다.

Raspberry Pi 5에서 BeamNG.drive 차량 데이터를 받아 볼스터 목표각을 정하고, NUCLEO-H723ZG(**OASIS_FALLBACK v2**)로 보냅니다.
- 펌웨어: `4_Seat_Actuation/stm32`, `-FallbackControl` 빌드
- 보드 프로토콜, 차량 행 검증, CAD 엔벨로프: `BeamNG_Bolster_FB`의 코드를 그대로 씁니다(`4_Seat_Actuation/experiment_a`, `fallback`)
- Pi는 PWM을 다루지 않습니다. 펌웨어가 IK, 한계, 변화율, 500 ms lease를 집행합니다.

```
BeamNG.drive → Collector v0.3 → fallback.beamng_bridge → beamng/pi_relay.py ──UDP 64257──┐   (PC, beamng/README.md)
                                                                                          ▼
Pi: beamng_rx (FB VehicleValidator) → 제어기 ─────────────────────────────────────► fb_link (FB BoardLink) ──USB──► STM32
        rl    : BeamNG 1 s 예측기 → obs(학습 계약) → SAC 잔차 액터 → 0.3 s LPF → 인터락 → 엔벨로프
        rule  : 같은 경로, 액터 대신 튜닝 규칙            (rl의 hot fallback, latched)
        fb_v6 : FB Experiment A 제어기 그대로 (차량 데이터만, V6 제어 법칙)   (모델이 없으면 cold fallback)
```

## 구성

| 경로 | 내용 |
|---|---|
| `runtime/oasis_rt/app.py` | **Pi 진입점.** 세션(정책 준비 → 보드 handshake → BeamNG 2 s 검증 → FB_ARM → 압력 기준선 → 제어 → OFF 확인), `--check-vehicle`, `--sil` |
| `runtime/oasis_rt/fb_link.py` | OASIS_FALLBACK v2 링크. FB `BoardLink` 위의 얇은 어댑터: 요청 스케줄, lease 권한, 압력 sweep |
| `runtime/oasis_rt/beamng_rx.py` | 릴레이 수신, Pi 시계 재타임스탬프, FB `VehicleValidator`, 조향 부호 감시 |
| `runtime/oasis_rt/envelope.py` | 모든 FB_SET 전 투영: CAD 결합 한계 ∩ STM 튜닝(yaw/pitch 상한) ∩ IK |
| `runtime/oasis_rt/controller.py` | 20 Hz RL / 규칙 제어기 (관측, 액터 백엔드, 인터락, hot fallback). `python -m oasis_rt.controller`는 이전 OASIS_CONTROL v6용입니다 |
| `runtime/oasis_rt/{actor,actor_core.c,predictor,safety,obs…}` | 액터 백엔드(C / numpy / Hailo), BeamNG 예측기, 인터락 |
| `runtime/oasis_rt/fbstack.py` | `4_Seat_Actuation` 위치 탐색과 import |
| `runtime/tests/` | `test_app_fb.py`(16개: OASIS_FALLBACK v2 모의 보드 + 합성 Collector 행), `test_runtime.py`(16개: 제어기·인터락·백엔드), `mock_fb_board.py`, `mock_stm32.py` |
| `runtime/config.yaml`, `runtime/systemd/oasis-rt.service`, `runtime/requirements.txt` | 설정, 서비스, 의존성 |
| `export/export_actor.py`, `export/make_calib_set.py` | 학습 npz → 모델 패키지(`model_v8/`: actor, 예측기, obs 계약, C 헤더, manifest + sha256, 학습 구동기) |
| `tools/pack_model.py` | 모델 패키지 → `dist/oasis_model_v8.tar.gz`, Pi에서 해시 검사 후 설치 |
| `tools/pad_calib.py`, `tools/PAD_CALIB_PROTOCOL.md` | 저울 패드 캘리브레이션 (인터락 counts/N) |
| [`CONTROL_SPEC.md`](CONTROL_SPEC.md) | 제어 사양 (관측, 예측기, 규칙, RL, 안전, fallback, 결과) |

## Pi 설치 (한 번)

```bash
# Raspberry Pi OS Bookworm (64-bit), 사용자 oasis
git clone https://github.com/xw00hw/2026ESWContest_mobility_O.A.S.I.S.git ~/ESW
ln -s "$HOME/ESW/3_RL_Control" ~/rl                 # systemd 경로에 공백이 없게
python3 -m venv ~/.venv/oasis && ~/.venv/oasis/bin/pip install -r ~/rl/deploy/runtime/requirements.txt
sudo usermod -aG dialout oasis                        # /dev/ttyACM0 (NUCLEO ST-LINK VCP)

# 모델 (git에 없음) -- PC: python deploy/tools/pack_model.py deploy/model_v8 ; scp dist/oasis_model_v8.tar.gz* oasis@<PI>:~/
cd ~/rl && ~/.venv/oasis/bin/python deploy/tools/pack_model.py --install ~/oasis_model_v8.tar.gz --into deploy/
cd deploy/runtime && make MODEL=../model_v8           # build/liboasis_actor.so (-mcpu=cortex-a76)
~/.venv/oasis/bin/python -m oasis_rt.actor --model ../model_v8 --backends c numpy   # 지연·오차, 자동 선택
~/.venv/oasis/bin/python tests/test_app_fb.py && ~/.venv/oasis/bin/python tests/test_runtime.py
~/.venv/oasis/bin/python -m oasis_rt.app --sil 60      # 장치 없이 세션 전체
```

- 모델 패키지가 없거나 깨지면 `control.cold_fallback: fb_v6`로 FB Experiment A 제어기가 세션을 맡습니다. 로그에 `COLD FALLBACK`이 찍힙니다.
- 네트워크: PC와 Pi를 **유선**으로 연결하세요(같은 스위치 또는 직결). `beamng.allow_from`에 PC 주소를 넣으면 다른 송신자를 버립니다.

## 보드 준비 (STM32)

1. `OASIS_FALLBACK v2` 이미지를 올립니다(`4_Seat_Actuation/stm32/FALLBACK_PROTOCOL.md`). **모터 전원을 끄고 합니다.**
2. STM Live Expressions(`EXPERIMENT_A_LIVE_TUNING.md`)로 이번 세션의 튜닝을 정합니다. 모터 OFF 상태에서 값을 쓰고 `request_seq`를 증가시킵니다.

| 값 | FB 기본 | RL 권장 | 이유 |
|---|---|---|---|
| `yaw_max_mdeg` | 15000 | **20000** | 정책 범위 20°. 15°로 두면 목표가 15°에서 잘립니다(v8은 15/20 둘 다로 학습) |
| `yaw_rate_mdeg_s` | 2000 | **20000** | v8은 10–20 °/s 튜닝으로 학습했습니다. 실제 속도는 2 µs/20 ms 서보 제한(약 5 °/s)이 결정합니다. 10 °/s 미만이면 런타임이 RL 대신 규칙을 씁니다 |
| `pitch_max_mdeg`, `pitch_rate_mdeg_s` | 6000, 2000 | 그대로 | v8은 pitch를 쓰지 않습니다(0°) |

3. Pi는 `#TUNE` 값을 세션 시작 때 고정합니다. 세션 중에 바뀌면 멈춥니다.
4. `#CAD` 해시와 HELLO(270°, 2 µs, lease 500 ms)가 맞아야 ARM합니다.

## 세션 실행

```bash
cd ~/rl/deploy/runtime
~/.venv/oasis/bin/python -m oasis_rt.app --check-vehicle 30     # BeamNG 스트림 + 조향 부호 확인 (STM32 안 엶)
# config.yaml confirmations 다섯 개를 사람이 확인하고 true로
~/.venv/oasis/bin/python -m oasis_rt.app                        # Ctrl+C = 정지 (FB_OFF + OFF 확인)
sudo systemctl start oasis-rt                                  # 또는 서비스로 (cp systemd/oasis-rt.service /etc/systemd/system/)
```

한 번 실행하면 세션 하나가 돌고 끝납니다. 오류가 나면 OFF하고 종료하며, 자동으로 다시 ARM하지 않습니다. 다시 하려면 원인을 본 뒤 명령을 다시 실행합니다.

1. 모델·정책 준비 → 시리얼·UDP 열기
2. 보드 handshake: HELLO v2 → OFF → CAD → TUNE → DISABLE → 압력 HELLO → START → STATE
3. 유효한 BeamNG 행과 PWM OFF가 **2 s 연속** → `FB_ARM` 한 번 → 중립(0°) armed STATE 확인 (0.5 s 안)
   - `FB_ARM`은 벤치 영점 펄스를 바로 냅니다. 기구가 움직일 수 있습니다(`neutral_start_clear`).
4. rl / rule: 중립에서 압력 기준선 2 s (착석, 볼스터 열림). 산포가 크면 시작을 거부합니다.
5. 제어 루프
   - rl / rule 20 Hz, fb_v6 50 Hz
   - PING 100 ms, STATE 100 ms, SET ≥ 40 ms, 압력 PING 1 s
   - **PING은 건강한 제어 tick 뒤에만 보냅니다.** Pi가 멈추면 500 ms 뒤 펌웨어가 스스로 PWM을 끕니다.

로그는 `~/oasis_logs/app_<시각>_hw_<고유값>/`에 남습니다. 같은 시각에 다시 시작해도 이전 세션을 덮어쓰지 않습니다.
- `manifest.json`: 결과, OFF 확인, 펌웨어 HELLO·TUNE, 엔벨로프, 모델 sha, FB 모듈 sha, 차량 행 통계, 조향 부호 근거
- `control.csv` (rl / rule): 관측 57개, 행동, 명령, 보고 각도, 인터락 힘
- `control_fb_v6.csv` (fb_v6)
- `pressure_raw.csv`, `pressure_sweeps.csv`: 세 모드 모두 FB V7과 같은 스키마로 12채널 raw ADC·status·채널별 MCU/호스트 시각을 보존합니다. nominal ADC 전압은 힘 보정값이 아닙니다.
- `pressure.jsonl`, `control.jsonl`: 압력 원본·스캐너 사건과 세션 생성 시계 정보. 압력 기록은 handshake에서 시작해 종료 시 DISABLE 확인까지 계속합니다.
- `INCOMPLETE`: writer가 모든 수락 기록을 flush/fsync하고 닫은 경우에만 삭제합니다. 기록 큐 손실·디스크 오류·닫기 시간 초과는 모터 권한을 중단하고 세션을 종료하며 마커와 manifest의 오류를 남깁니다. 기록 준비는 UDP/시리얼 열기보다 먼저 수행합니다.

Collector 원본은 PC에 따로 남습니다(FB와 같음).

## v9 체형 스케일 — 다른 사람이 앉을 때 (기본 OFF)

10/28 착석 G 제어기(G 기본 곡선 + RL ±1.5° 보정)는 기준 사용자(10/28 최종 세션 착석자)에게 맞춰져 있습니다. 다른 사람이 앉으면 카메라 중립에서 잰 체형 비율로 **G 기본각·예측각만** s배 합니다. s는 0.9–1.1입니다. 상한(20° / 예측만 10°), RL 보정 폭, 압력 해제, CAD·STM 한계는 그대로입니다. 식과 튜닝 근거는 [`CONTROL_SPEC.md` §4b](CONTROL_SPEC.md)에 있습니다.

1. 카메라·좌석 위치를 10/28 구도 그대로 둡니다. 기준값이 화면 대비 비율이라, 구도가 바뀌면 체형이 바뀐 것처럼 보입니다.
2. 새 착석자로 카메라 중립을 새로 수집합니다(`experiments/seated_20261028/camera/collect_pose_reference.py`, 작업자 확인). 이 파일을 `pose_camera --neutral`과 같이 씁니다.
3. 실행 설정에 다음을 넣습니다. `g_residual_control.enabled: true`가 함께 필요합니다.
   ```yaml
   body_scale: {enabled: true, neutral: /absolute/this_person_neutral.json}
   ```
4. 시작 로그에서 `body scale 0.9xx (shoulder ratio …, torso ratio …)`를 확인합니다. 같은 값이 `manifest.json`의 `body_scale`과 매 tick의 `g_control_tick.governor.body_scale`에 남습니다.

- 기준 사용자 본인이거나 카메라를 쓰지 않으면 `enabled: false`로 둡니다(s = 1, 10/28과 같은 출력).
- 켰는데 중립 파일이 없거나 깨졌거나, 비율이 0.7–1.4를 벗어나면 시작을 거부합니다. 조용히 1.0으로 돌지 않습니다.
- 세션 중에는 s가 바뀌지 않습니다. 사람이 바뀌면 OFF 후 새 중립과 새 세션으로 시작합니다.

## 향후 카메라 pose 입력 — 현재 기본값은 OFF

### 카메라 미리보기 (2026-10-28 로컬 준비, Pi 미배포)

`python -m oasis_rt.pose_camera`에 `--preview-port 8080`을 추가하면 같은 capture/inference 결과로 키포인트·ROI가 표시된 MJPEG와 `/status.json`을 제공합니다. 기본값 0은 미리보기 OFF입니다. 서버는 `127.0.0.1`에만 바인딩하며 PC에서는 SSH 로컬 터널을 사용합니다. 별도 카메라/추론 인스턴스, 영상 녹화, 모터 조작은 추가하지 않습니다.

최신 프레임 한 개를 별도 스레드에서 인코딩하고 HTTP 클라이언트는 최대 4개로 제한합니다. 캡처 시각이 오래되거나 카메라가 끊기면 서버와 브라우저에서 이전 영상을 가립니다. 정상 polling 공백은 원래 캡처 시각의 TTL 안에서만 영상을 유지합니다. 미리보기 오류는 기존 optional pose producer와 중립/신뢰도 판정을 바꾸지 않습니다. 중립 미보정 상태에서도 현재 관절을 볼 수 있지만 RL 유효 자세로 취급하지 않습니다.

오프라인 preview 검사 15개와 합성 JPEG 표시를 확인했습니다. 카메라 구도·중립·실제 Hailo 추론 FPS·Pi 연결은 이 검사에 포함되지 않았습니다. 모델 파일과 기존 Hailo hybrid qualification 계약은 그대로 필요합니다.

`oasis_rt.app`은 `pose.source: none`, `profile.from_camera: false`가 기본입니다. 이때 카메라·pose 소켓을 열지 않고 관측 37/38은 0, 54는 1 s로 유지합니다. 선택형 `pose.source: udp`는 같은 Pi의 **전처리된 feature**만 `127.0.0.1:<pose.udp_port>`에서 받으며 카메라나 DAQ를 실행하지 않습니다. 사용할 때는 기대하는 `pose.source_id`를 명시해야 합니다. 기존 `oasis_rt.controller`의 legacy UDP 경로는 이 새 계약의 대상이 아닙니다.

`oasis.pose.v1` JSON은 아래 필드만 받습니다. 아래 시각은 형식 예시이며 실제 송신기는 자신의 캡처 시각을 넣어야 합니다.

```json
{"schema":"oasis.pose.v1","source_id":"front_seat_pose","stream_id":"camera-session-1","seq":1,"capture_clock":"monotonic","capture_ns":123456789000,"torso_lean_deg":2.5,"lean_score":0.05,"pose_confidence":0.9,"vision_valid":true,"neutral_valid":true}
```

- `capture_clock`: `monotonic` 또는 지원되는 `monotonic_raw`. 같은 Pi의 해당 시계로 캡처 나이를 계산하며 오래된 프레임을 수신 시각으로 갱신하지 않습니다. 미래 시각·중복/역순 sequence·캡처 시각은 거부합니다. producer 재시작 시 새 `stream_id`를 사용합니다.
- `torso_lean_deg`: 중립 대비 각도, 유한한 ±90° 이내. `lean_score`: 중립 어깨폭으로 정규화한 횡이동, 유한한 ±4 이내. `pose.lean_sign`은 확인된 이미지→차량 부호입니다. 이는 패킷 유효 범위이며 착석 구동의 안전 승인이 아닙니다.
- 두 validity 필드는 `true`, confidence는 0–1 및 `pose.min_confidence` 이상이어야 합니다. 잘못되거나 `pose.stale_s` 이상 지난 입력은 pose 미사용(관측 0, 나이 1 s)으로 처리합니다. 기록 오류는 기존 세션 OFF·종료 규칙을 따릅니다.
- 읽기량은 한 번에 최대 32개와 backlog 확인 1개, datagram은 4096 bytes로 제한합니다. 수락 feature·거부 이유는 기존 bounded `control.jsonl`, source/clock/통계는 `manifest.json`의 `pose`에 남습니다.

DAQ 원본은 직접 연결하지 않습니다. DAQ `torso_angle_deg`에는 중립 각도 차감이 필요하고, `shoulder_width`는 이미지 정규화값입니다. 시계·부호·중립 기준을 맞추는 producer adapter는 별도 작업입니다. 보정된 미터 단위 체형 인터페이스가 준비되기 전에는 `profile.from_camera: true`를 거부합니다. 카메라 OFF 기본 경로와 주입된 feature/시계를 이용한 SIL만 검증하며, 실카메라 연결이나 효과를 검증한 것은 아닙니다.

## 실패하면 무엇이 일어나나

| 상황 | 동작 |
|---|---|
| ARM 전: 잘못된 행, 끊김, 보드 OFF 미확인, 조향 부호 불일치 | 2 s 검증을 처음부터 다시 (ARM 안 함) |
| ARM 후: 잘못된 행, 행 끊김 > 200 ms, 순서 어긋남 | `vehicle:<이유>` → FB_OFF → 종료 |
| ARM 후: 충격 의심(FB V7 `ImpactDetector`: 평면 가속도 ≥ 25 m/s²가 0.06 s 이상 + 급변 ≥ 150 m/s³ + 구간 적분 ≥ 1.5 m/s, 또는 ≥ 60 m/s² 즉시) | `vehicle:impact:<이유>` → FB_OFF → 종료. 모든 모드. 한 샘플의 큰 a_y만으로는 멈추지 않고 상한에 포화합니다(FB ADR-032) |
| ARM 후: 조향 부호 불일치 (rl / rule, 코너 근거 100행 이상, 일치율 < 20 %) | `steer_sign_mismatch` → FB_OFF → 종료 (`beamng.steer_sign` 반대로) |
| 액터 예외 / NaN | 튜닝 규칙으로 전환, 세션 끝까지 유지 (hot fallback) |
| 예측기 NaN | 그 tick만 현재 a_y 유지(hold)로 대체 |
| 모델 패키지 없음 / 깨짐 | FB Experiment A 제어기 (cold fallback), 설정이 null이면 시작 거부 |
| 모델이 다른 구동기용 (v7) / STM 튜닝이 학습보다 느림 | 튜닝 규칙으로 세션 (`control.actuator_mismatch: rule`) |
| 보드 `#ERR`, fault, `#BOOT`, ACK 200 ms 초과, Pi poll 간격 > 100 ms | BoardLink가 latch → 비상 FB_OFF → 종료 |
| Pi가 멈춤 | PING이 끊기고 500 ms 뒤 펌웨어 LEASE → PWM OFF |
| 목표가 엔벨로프 밖 | Pi가 투영합니다. 펌웨어에는 CAD / TUNE 밖 목표가 가지 않습니다(밖이면 펌웨어가 OFF) |

PWM OFF는 신호 상태일 뿐입니다. 하중이 걸린 기구가 풀리거나 제자리에 있다는 보장이 아닙니다(FB 문서와 같은 주의).

## Arducam / RTMPose 선택형 프로세스

`oasis_rt.pose_camera`는 제어기와 분리된 카메라 프로세스입니다. 최신 프레임 하나만 유지하고 V4L2의 촬영 시각을 사용합니다. 카메라 없음·점유·끊김·낮은 관절 신뢰도·오래된 결과·중립 미보정이면 유효한 자세를 보내지 않습니다. 제어기는 pose 특징을 unavailable로 처리하고 BeamNG·압력 경로를 계속 사용합니다. 촬영 프로세스는 UART나 모터 인터페이스를 열지 않습니다.

```bash
python -m oasis_rt.pose_camera --backend onnx-reference --model /path/to/rtmpose_m.onnx \
  --device /dev/v4l/by-id/<Arducam-video-index0> --source-id arducam-rtmpose \
  --duration 10 --status /path/to/new_camera_status.json
```

이 명령은 CPU 기준 구현 검증이며 Hailo 가속이 아닙니다. Hailo 선택 시 호환 HEF/백엔드가 없으면 unavailable로 남고 CPU로 몰래 전환하지 않습니다. 중립 파일은 명시적인 작업자 확인과 모델 해시·프레임 크기·ROI·카메라 ID가 맞아야 사용됩니다. 재연결 뒤에는 중립을 무효화합니다. 이미지에서 미터 단위 체형을 추정하는 기능은 켜지 않습니다.

`--backend hailo-hybrid --hybrid-manifest PATH`는 Hailo10H 앞부분과 원본 ONNX CPU 뒷부분을 연결하는 선택 경로입니다. 정확한 파일 해시로 고정한 두 경계만 허용합니다. `Conv_205:625`는 CPU 입력 `[1,17,8,6]`, `Mul_204:624`는 CPU 입력 `[1,768,8,6]`이며, 후자는 마지막 7×7 합성곱과 SimCC head를 CPU에서 계산합니다. 서로 다른 경계의 파일·배열을 섞으면 장치를 열기 전에 거부합니다. manifest에는 실제 HEF·head 해시, HailoRT 5.4.0, 장치 주소, 입출력 배열과 양자화 정보를 지정해야 합니다. 기존 `--model` 옵션과 함께 사용하지 않습니다. Hailo 입출력은 정규화된 BGR FLOAT32/NHWC이며 HailoRT가 양자화와 역양자화를 처리합니다. CPU head 앞에서 NCHW로 되돌립니다. 파일·장치 부재, 계약 불일치, 추론 시간 초과는 유효 pose를 만들지 않습니다.

DFC 5.4.0에서 전체 모델은 head의 MatMul 변환에 실패했고 Conv_205 앞부분의 파싱은 성공했습니다. 분리한 부동소수점 ONNX의 18개 입력 비교에서 최대 SimCC 차이는 약 1.30e-6이었으며 신뢰도 높은 어깨·골반 좌표는 같았습니다. 이는 양자화된 Hailo 출력의 정확도나 실제 가속 성능 검증이 아닙니다. 현재 카메라 위치와 중립이 준비되기 전에는 pose 입력을 활성화하지 않습니다.

2026-10-28 후속 변환: Conv_205 경계의 양자화 비교는 통과했지만 실제 컴파일에서 마지막 7×7 합성곱의 Hailo 자원 배치가 실패했습니다. Mul_204 경계로 옮긴 부동소수점 분리는 50개 입력에서 원본과 일치했고, Pi에서 CPU 뒷부분만 실행한 32개 입력 비교도 통과했습니다(최대 SimCC 차이 1.42e-6, 배열 변환 포함 중앙값 4.42ms). 이는 원본 모델과의 수치 비교이며 사람의 실제 자세 각도 정확도 검증은 아닙니다. 두 경계는 서로 다른 중립 보정을 사용해야 합니다.

2026-10-27 Pi 5/Arducam OV9281 실측: CPU RTMPose 60프레임/10.21초(5.88fps), 평균 추론 136ms. 카메라가 천장을 향해 유효한 몸통 관절이 없었으므로 유효 pose와 중립 보정은 생성하지 않았습니다. 카메라 미연결 시험도 종료·자원 해제됐습니다. 이 수치는 Hailo 성능이 아닙니다. 이후 Hailo-10H 드라이버·펌웨어·HailoRT·Python 5.4.0 설치와 제조사 진단 모델의 실제 추론을 확인했습니다. Python FLOAT32/NHWC 바인딩도 실제 장치에서 확인했으며 RTMPose의 양자화 정확도와 속도는 별도로 검증해야 합니다.

HailoRT 5.4의 실제 검색 ID는 `pci/0000:01:00.0` 형식입니다. hybrid backend는 전체 PCI 주소를 대조하고 검색된 식별자를 그대로 장치 생성에 사용합니다. 로컬 PCI 주소의 `pci/` 접두어 생략도 허용하며 네트워크·USB 식별자는 받지 않습니다.

MOZA 직접 조향의 별도 수신·보정·예측 우선순위는 [BeamNG 안내](../beamng/README.md#선택-입력-moza-직접-조향)를 따릅니다. 기본 설정에서는 카메라와 MOZA 모두 비활성입니다.

## 성능 메모

- 액터 C 백엔드: Ryzen 중앙값 6.9 µs. Pi 5 예상 20–40 µs. tick 전체 < 1 ms / 50 ms. Hailo는 이 MLP에서 이득이 없어 기본에서 뺐습니다.
- 실제 지연은 연산이 아니라 **구동기**가 정합니다.
  - OASIS_FALLBACK v2는 서보 펄스를 20 ms마다 2 µs까지만 바꿉니다(크랭크 13.5 °/s, 구조 yaw 약 5–6 °/s).
  - yaw 0→9°에 1.5 s가 걸립니다(튜닝 10–20 °/s). 튜닝 2 °/s(FB 기본)에서는 4.5 s입니다(`sim/v3/audit_v6.py` 6b).
  - 그래서 v8은 이 구동기로 다시 학습했습니다. v7은 더 빠른 가정(크랭크 30–60 °/s)이었고, 이 구동기에서는 규칙보다 나빴습니다(`sim/v3/MODEL_CARD_v8.md`).
  - v8 RL은 이 구동기에서 튜닝 규칙과 같은 수준입니다(평가 은행 J 3.92 vs 3.93, FB V6 1.70). L2 벤치 체감이 RL 쪽이 나쁘면 `control.mode: rule`로 둡니다. 시뮬레이터 성능은 같습니다.
# 2026-10-28 로컬 후보: G 기본각 + RL 보정

현재 작업 브랜치는 `oasis_rt.g_residual_control`을 포함한다. 새 시험 프로필에서 활성화하면
측정0.0625g→10°, 1g→20° 기본각에 학습 prior 대비 actor 유효각 차를 최대±1.5°로 더한다.
1g에서 보정은0°, 예측만의 상한은10°, Pitch는 yaw-only 모델/시험 wrapper에서0°다.
기본 `config.yaml`에서는 비활성이므로 기존 배포가 자동으로 바뀌지 않는다.

`g_control_tick` JSONL은 기본각·보정·필터전후·최종전송큐·보드명령상태를 기록한다.
PC `beamng/rl_monitor.py`는 이 로그를 관찰만 한다. 예: `python rl_monitor.py --ssh-host PI_HOST --ssh-log /absolute/run/control.jsonl --open-browser`.
Pi monotonic clock은 차트축에만 사용하며 PC수신시각으로 표시 신선도를 판정한다.
실행 중 받은 새 행만 LIVE로 읽으며 재생은 `--replay FILE`로 명시한다.

이 후보는 기존 raw2500 착석시험 wrapper와 함께 사용해야 한다. generic app 설정 자체가
raw2500 latch를 대신하지 않는다. 로컬 `MOTER/build/rl_main_20261027/g_hybrid_tuning_20261028`
의 후보/검증기록을 참고한다. 현재 Pi는 분리되어 있어 새 후보는 배포되지 않았다.
