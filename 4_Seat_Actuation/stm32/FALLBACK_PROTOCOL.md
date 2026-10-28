[← STM32H723 볼스터 · 압력 통합 펌웨어](README.md)

# OASIS_FALLBACK v2 펌웨어 계약

> 최종 RL·백업 제어에 쓰는 선택 빌드 `OASIS_FALLBACK_CONTROL=ON`의 출력 동작, UART 프로토콜, 상태·유효시간, 안전 장치를 정리합니다.
> 부팅 시 PWM은 꺼져 있고, 호스트가 새 세션 ID로 명시적으로 `FB_ARM`해야만 출력합니다.

---

## 1. 빌드

| 항목 | 내용 |
|---|---|
| CMake 옵션 | `OASIS_FALLBACK_CONTROL=ON` (기본 OFF). 측정용 통합 v6와 별도 이미지 |
| 배타 조건 | `OASIS_PRESSURE_ONLY`와 동시에 켜면 빌드 거부 |
| 결과물 | `build_fallback/fallback_control_h723.elf`, `.bin` (저장소 미포함) |
| 빌드 명령 | `scripts/stm32.ps1 -Action Build -FallbackControl` (CMake·Ninja·GCC·SDK 경로를 인자로 전달) |

## 2. 출력과 보정

| 항목 | 내용 |
|---|---|
| 부팅 | 첫 애플리케이션 동작에서 TIM1을 끄고 D3/D4/D5/D6를 GPIO LOW로 설정. 리셋부터 이 코드 실행 전까지의 핀 상태는 소프트웨어로 보장하지 않음 |
| 핀 | D3/D4/D5/D6 = PE13/PE14/PE11/PE9 = TIM1 CH3/CH4/CH2/CH1, AF1, 50 Hz. MUX D8~D11, EN D12, ADC A0/A1/A2는 압력 스캔과 공유 |
| ARM 시 영점 | D3/D4/D5/D6 = **1800 / 1800 / 1200 / 1200 µs** (현재 장착 기준 보정 영점, `JOINT_BOOT_PULSES`) |
| 환산 | `joint_math.c`의 C4_R8 역기구학, DS5180SSG 270°, 2000 µs / 270° |
| 펌웨어 고정 상한 | 구조물 yaw 0~27°, pitch ±8°, 펄스 500~2500 µs, IK 도달 조건 |
| CAD 간섭 한계 | yaw 최대 22°, pitch 최대 ±6°. Pitch 상한은 yaw 0°→0°, 6~15°→±6°, 20°→±2°, 22°→0° 사이 선형 보간 |
| 운용 상한 | Live Expressions 튜닝값 (기본 yaw 15°, pitch ±6°) → [실험 A 실시간 튜닝](EXPERIMENT_A_LIVE_TUNING.md) |

- **최초 ARM 출력**은 2 µs 변화 제한의 예외입니다. 영점 펄스로 바로 출력하므로 실제 축이 움직일 수 있습니다. 호스트는 장치·장착·보정 프로필 일치와 초기 위치 적용 승인을 먼저 확인해야 합니다. 측정용 v6 이미지의 영점(1800 / 1200 / 1200 / 1800 µs)과 그 기록은 그대로 유지됩니다.
- 영점 펄스는 비어 있는 타이머 프레임 뒤 자연 overflow에서 네 채널이 동시에 시작합니다. 이후 CCR 갱신은 자연 프레임당 최대 1회, 모든 허용 펄스가 끝난 CNT 3000~17000 µs 구간에서만 준비합니다. UDIS로 네 preload를 묶고, 동작 중 UG를 발생시키지 않습니다.
- 목표가 바뀌어도 20 ms 추종 주기는 다시 시작하지 않습니다. 각 갱신은 이전 명령 대비 축당 최대 2 µs이며, 늦어진 주기에서 밀린 변화를 몰아서 수행하지 않습니다. 튜닝값의 각속도 상한도 함께 적용합니다.
- `FB_OFF`와 fault는 즉시 PWM을 중단하고 GPIO LOW로 전환합니다. 진행 중 펄스가 잘릴 수 있으며 자동 중립 복귀는 하지 않습니다. 외부 모터 전원을 차단하는 기능은 아닙니다.

## 3. ASCII 프로토콜

460800 bps, 8N1, LF 종결. 정수 필드에는 공백, `+`, 지수, 소수, 빈 필드를 허용하지 않습니다. 양의 정수 ID 범위는 1~4294967295이며, 음수는 pitch 필드에만 허용됩니다.

```text
CONTROL_HELLO
#HELLO,OASIS_FALLBACK,v2,yaw_min_mdeg=0,yaw_limit_mdeg=27000,pitch_limit_mdeg=8000,servo_range_deg=270,neutral_us=1800:1800:1200:1200,boot_pwm=OFF,lease_ms=500,tick_ms=20,max_step_us=2,ds_outputs=4,ik_revision=8,session_floor=<n>

CAD_INFO
#CAD,OASIS_A,v2,profile_sha256=<CAD 프로필 SHA256>,yaw_max_mdeg=22000,pitch_max_mdeg=6000,guard=CAD_KINEMATIC

TUNE_STATE
#TUNE,OASIS_A,v1,revision=<r>,yaw_max_mdeg=<..>,pitch_max_mdeg=<..>,yaw_rate_mdeg_s=<..>,pitch_rate_mdeg_s=<..>,full_scale_ay_mmps2=<..>,pitch_scale_permille=<..>

FB_ARM,<session_id>,<tuning_revision>
#ACK,FB_ARM,<session_id>,<tuning_revision>

FB_SET,<session_id>,<command_id>,<left_yaw_mdeg>,<left_pitch_mdeg>,<right_yaw_mdeg>,<right_pitch_mdeg>
#ACK,FB_SET,<session_id>,<command_id>,<left_yaw_mdeg>,<left_pitch_mdeg>,<right_yaw_mdeg>,<right_pitch_mdeg>

FB_PING,<session_id>,<counter>
#ACK,FB_PING,<session_id>,<counter>

FB_OFF
#ACK,FB_OFF
```

호스트(`experiment_a/transport.py`)는 연결할 때 HELLO의 고정 필드, `CAD_INFO` 응답 전체, `TUNE_STATE`를 확인하며, 영점이나 버전이 다른 HELLO는 거부합니다. `CAD_INFO`와 `TUNE_STATE`는 motor lease를 갱신하지 않습니다.

### 세션 규칙

| 규칙 | 내용 |
|---|---|
| `session_floor` | 부팅 후 마지막으로 수락한 세션 ID. 새 ARM은 이보다 큰 ID가 필요 |
| 세션 재사용 | 현재 세션 재ARM, 예전 세션 재사용, ARM 중 ARM 모두 거부 (ARM 상태면 출력 OFF) |
| 재부팅 | floor가 0으로 돌아가므로 호스트는 MCU 재부팅을 감지하고 **자동 재ARM하지 않음** |
| ID 소비 | ARM 도중 하드웨어 오류가 나도 그 ID는 소비됨 |
| 튜닝 revision | `FB_ARM`의 revision이 현재 적용 revision과 다르면 `TUNE_REVISION`으로 거부 |
| SET ID · PING counter | 세션 안에서 각각 엄격히 증가. 0에서 시작하므로 첫 요청은 1 이상 |
| ACK 의미 | 명령 수락. 실제 위치 도달 확인이 아님 |
| SET | 최신 목표만 교체. 누적 이동이나 반복 왕복을 만들지 않음 |

### 오류 응답

형식: `#ERR,<command_id 또는 0>,<reason>`. ARM 상태에서 오류가 나면 `PROTOCOL` fault로 출력을 끄고 새 세션을 요구합니다.

| reason | 의미 |
|---|---|
| `INVALID` | 필드 형식·범위 오류 |
| `DISARMED` | ARM되지 않은 상태의 SET/PING |
| `SESSION` | 현재 세션과 다른 ID |
| `SESSION_REPLAY` / `SET_REPLAY` / `PING_REPLAY` | 세션·명령·counter 재사용 |
| `ALREADY_ARMED` | ARM 중 ARM |
| `TUNE_REVISION` | ARM 요청의 튜닝 revision 불일치 |
| `CAD_BOUNDS` / `TUNE_BOUNDS` | CAD 간섭 한계 / 운용 상한 밖 목표 (출력 OFF) |
| `MOTOR_BOUNDS` | 역기구학·펄스 범위로 표현할 수 없는 목표 |
| `HARDWARE` / `WATCHDOG` | PWM 레지스터 이상 / watchdog 미준비 |
| `UNSUPPORTED` | 일반 `ARM`, `SET`, `HOLD` 등 호환 명령 (처리하지 않음) |

## 4. 상태와 유효시간

`STATE` 응답과 명목 5 Hz 상태 보고는 18필드입니다.

```text
#STATE,mcu_ms,armed,moving,LY,LP,RY,RP,target_LY,target_LP,target_RY,target_RP,D3_us,D4_us,D5_us,D6_us,command_id,fault
```

- 각도는 mdeg, 펄스는 µs입니다. 부팅 직후에는 모든 수치가 0입니다.
- OFF 후에는 마지막 명령 자세가 남고 펄스는 모두 0이며, target은 마지막 명령 자세로 정리됩니다. 따라서 v6의 "모든 상태에서 자세와 PWM 일치" 파서를 그대로 쓸 수 없습니다.
- 모든 값은 **명령 상태**이며 실제 위치 측정값이 아닙니다.

### Motor lease

| 항목 | 내용 |
|---|---|
| 갱신 | 유효한 `FB_ARM` / `FB_SET` / `FB_PING`만 갱신. 압력용 `PING`은 갱신하지 않음 |
| 만료 | 마지막 갱신 후 500 ms 이상임을 다음 검사에서 확인하면 `LEASE` fault → GPIO LOW |
| 검사 시점 | 송신 대기 중, 블로킹 송신 전후, ADC 뱅크 사이, 명령 처리 전후 |
| 추가 지연 | 한 HAL 호출의 블로킹 시간(UART timeout 20 ms, ADC 변환 timeout 5 ms)과 실행 지연이 더해질 수 있음. 실제 최대 지연은 미측정 |

500 ms는 검사 기준이며 정확히 500.000 ms에 하드웨어가 차단된다는 보장은 아닙니다. 압력 스캔의 3초 watchdog과 ADC/UART 오류 경로는 유지되며, 해당 오류도 fallback 출력을 끕니다. `DISABLE`은 압력 스캔만 종료하므로 전체 종료 시에는 `FB_OFF`와 `DISABLE`을 각각 확인합니다.

### Fault 값

| fault | 의미 |
|---|---|
| `BOOT` | 부팅 직후 |
| `NONE` | 정상 ARM |
| `OFF` | 정상 `FB_OFF` |
| `LEASE` | lease 만료 |
| `PROTOCOL` | ARM 중 프로토콜 오류 |
| `CAD_BOUNDS`, `CAD_PATH`, `TUNE_BOUNDS` | CAD 한계·경로 / 운용 상한 위반 |
| `PATH_BOUNDS`, `RATE_BOUNDS` | 경로 계산 실패 / 변화율 위반 |
| `HARDWARE`, `WATCHDOG` | PWM 레지스터 이상 / watchdog 시작 실패 |
| `FSR_FAULT`, `UART_FAULT` | 압력 스캔·UART 오류 |

`FB_OFF`는 기존 fault를 지우지 않습니다. 새 세션의 ARM을 명시적으로 수락할 때만 해제됩니다.

## 5. 추가 안전 장치

| 장치 | 내용 |
|---|---|
| RX 제한 | poll당 최대 32바이트·완성된 행 1개. dispatch 전에 lease 만료 확인 |
| 레지스터 확인 | PWM 갱신 전 readback 검증. 손상된 CCR를 새 값으로 덮어 숨기지 않음 |
| IWDG1 | LSI /64, reload 999 (32 kHz 가정 시 명목 2초). 압력 스캔·제어 한 반복이 끝났을 때만 갱신 |
| 디버그 | TIM1·IWDG debug freeze 해제. CPU halt 시 즉시 OFF되지는 않으며 watchdog 리셋까지 기존 펄스가 남을 수 있음 |

ARM 상태에서 디버거로 halt·step 하는 것은 정상 운용 방식이 아닙니다.

## 6. 검증

`scripts/test_fallback_elf.py`는 실제 생성 ELF를 Unicorn 에뮬레이터에서 실행합니다.

| 시험 그룹 | 검사 내용 |
|---|---|
| 제어 (`ControlTests`) | 부팅 OFF, 일반 명령으로 ARM 불가, 세션·명령 재사용, 5 ms 목표 스트림 아래 추종과 2 µs 한계, lease 만료, 압력 PING 분리, RX 제한, 양쪽 목표 도달·재목표 |
| 튜닝 (`TuningTests`) | 원자적 적용·새 sequence 요구, 구동 중 거절, ARM revision 경합, 정수 범위, 운용 상한과 CAD 상한 독립 |
| 레지스터 (`RegisterTests`) | watchdog 레지스터·완전한 반복에서만 갱신, 첫 프레임 0 후 영점, CCR마다 UDIS 묶음, 고구간 거부, OFF·손상 시 전 출력 LOW |
| CAD (`CadEnvelopeTests`) | HELLO 영점 = ARM 영점, 270° 이득, 호스트와 크랭크각·펄스 일치, 22,001개 정수 yaw 경계, CAD 거부 시 전 출력 OFF |

레지스터·통신 시험은 Cortex-M7 모델에서 실제 출력·프로토콜·IWDG 코드를 실행하고, double-precision 명령 제약 때문에 수학·연속 추종 시험은 ARM_MAX 모델과 PWM 경계 stub을 사용합니다.

> 실물 파형, 축 위치, 힘, 간섭, 발열, 시간 지연은 이 시험으로 검증하지 않습니다.
