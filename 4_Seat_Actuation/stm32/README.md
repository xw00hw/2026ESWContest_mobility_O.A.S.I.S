[← 4. 시트 구동](../README.md) &nbsp;|&nbsp; ① 센싱 → ② 처리 → ③ RL 제어 → **④ 시트 구동**

# STM32H723 볼스터 · 압력 통합 펌웨어

> NUCLEO-H723ZG 한 보드에서 **서보 4개 PWM, 압력 12채널 ADC 스캔, UART 명령 처리**를 함께 수행합니다.
> 최종 제어에 쓰는 `OASIS_FALLBACK v2` 빌드는 부팅 시 출력을 끈 상태로 시작하고, 호스트가 세션을 명시적으로 열어야만 움직입니다.

---

## 소스 구성

| 파일 | 역할 |
|---|---|
| `Core/Src/main.c` | 초기화, 메인 루프 (압력 스캔과 제어를 번갈아 처리) |
| `Core/Src/fallback_control.c` | **최종 제어 프로토콜**: 세션 ARM, 목표 설정, lease, fault 처리 |
| `Core/Src/fallback_pwm.c` | TIM1 50 Hz PWM, 네 채널 동시 갱신, 프레임당 2 µs 변화 제한 |
| `Core/Src/fallback_geometry.c` | CAD 간섭 한계 검사 (`fallback_geometry_profile.h`, PC와 같은 SHA256) |
| `Core/Src/fallback_tuning.c` | 각도·속도 상한 튜닝 값 (디버거 Live Expressions로 안전하게 변경) |
| `Core/Src/joint_math.c` | 역기구학: 구조물 Yaw/Pitch → 크랭크 각도 → 서보 펄스 |
| `Core/Src/fsr_single.c` | MUX 3개 × 4채널 압력 스캔, 압력 프로토콜 |
| `Core/Src/joint_control.c` | 측정 스테이션용 제어 (OASIS_CONTROL v6) |
| `Core/Src/pressure_only_control.c` | 압력 전용 빌드 |
| `scripts/stm32.ps1` | 빌드·장치 목록·업로드 |
| `scripts/test_fallback_elf.py`, `test_pressure_only_elf.py` | 빌드된 ELF를 Cortex-M7 에뮬레이터(Unicorn)로 검사 |
| `scripts/generate_fallback_geometry.py` | CAD 프로파일(`config/experiment_a_cad_profile.json`) → C 헤더 생성 |

## 동작 원리 (OASIS_FALLBACK v2)

```text
부팅 ──► PWM OFF (GPIO LOW)
          │  FB_ARM,<새 세션 ID>,<튜닝 revision>
          ▼
       중립 PWM 출력 (D3/D4/D5/D6 = 1800/1800/1200/1200 µs)
          │  FB_SET,<세션>,<ID>,<좌 yaw>,<좌 pitch>,<우 yaw>,<우 pitch>   (mdeg)
          ▼
       CAD 한계·도달 범위 검사 ─► 역기구학 ─► 20 ms마다 축당 최대 2 µs씩 목표로 이동
          │
          ├─ FB_PING이 500 ms 동안 없음 ─► LEASE fault ─► PWM OFF
          ├─ 잘못된 세션·명령 재사용·범위 밖 ─► fault ─► PWM OFF
          └─ FB_OFF ─► PWM OFF (자동 중립 복귀 없음)
```

- **세션 ID는 계속 증가해야** 합니다. 이전 세션을 다시 쓰거나 같은 세션으로 재ARM할 수 없어, 호스트가 재시작돼도 오래된 명령이 실행되지 않습니다.
- **ACK는 수락 사실**이지 실제 위치 도달 확인이 아닙니다. 보드가 보고하는 각도는 모두 명령 값입니다(위치 센서 없음).
- **압력 PING은 모터 lease를 갱신하지 않습니다.** 압력 스캔이 살아 있어도 제어 명령이 끊기면 모터는 꺼집니다.
- 독립 watchdog(약 2초)은 압력·제어 반복이 모두 끝났을 때만 갱신합니다.

## UART 프로토콜

460800 bps, 8N1, LF로 끝나는 ASCII. 전체 사양: [`FALLBACK_PROTOCOL.md`](FALLBACK_PROTOCOL.md)

```text
CONTROL_HELLO                                    → #HELLO,OASIS_FALLBACK,v2,...,neutral_us=1800:1800:1200:1200,lease_ms=500,...
FB_ARM,<session_id>,<tuning_revision>            → #ACK,FB_ARM,...
FB_SET,<session_id>,<cmd_id>,<LY>,<LP>,<RY>,<RP> → #ACK,FB_SET,...       (각도 mdeg)
FB_PING,<session_id>,<counter>                   → #ACK,FB_PING,...
FB_OFF                                           → #ACK,FB_OFF
STATE                                            → #STATE,mcu_ms,armed,moving,LY,LP,RY,RP,target...,D3_us..D6_us,command_id,fault
START_12,15 / PING / DISABLE                     → 압력 스캔 시작 / 유지 / 종료
```

오류는 `#ERR,<id>,<reason>`이며, reason은 `INVALID`, `SESSION`, `SET_REPLAY`, `MOTOR_BOUNDS`, `WATCHDOG` 등입니다.

## 빌드 종류

| 빌드 | 옵션 | 결과물 | 용도 |
|---|---|---|---|
| **OASIS_FALLBACK v2** | `-FallbackControl` | `build_fallback/fallback_control_h723.elf` | **최종 RL·백업 제어** |
| OASIS_CONTROL v6 | (기본) | `build/fsr_mux_h723.elf` | 측정 스테이션 ([`gui/`](../gui/README.md)) |
| PRESSURE_ONLY | `-PressureOnly` | `build_pressure_only/pressure_only_h723.elf` | 압력 센서 위치 확인 (서보 출력 차단) |

## 빌드 · 업로드

필요 도구: STM32CubeIDE ARM GCC, STM32CubeH7 **V1.13.0**, CMake ≥ 3.20, Ninja, STM32CubeProgrammer.
HAL/CMSIS와 startup·linker 파일은 설치된 CubeH7 SDK에서 가져옵니다. 도구 경로가 다르면 `-CMakePath`, `-NinjaPath`, `-ToolchainPath`, `-CubePath`, `-ProgrammerPath`로 지정합니다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/stm32.ps1 -Action Build -FallbackControl
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/stm32.ps1 -Action List
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/stm32.ps1 -Action Upload -StLinkSerial '<ST-LINK serial>'
```

업로드는 대상 ST-LINK serial을 명시해야 실행되고, 쓰기 후 검증·리셋까지 수행합니다(`Download verified successfully`).
**모터 전원을 끈 상태에서** 업로드합니다.

## 튜닝 (각도·속도 상한)

각도·속도 상한은 CubeIDE Live Expressions의 `g_experiment_a_tune`에서 바꿉니다. 모터 출력이 꺼져 있을 때만 적용되고,
`request_seq`를 증가시키면 펌웨어가 전체 값을 검증한 뒤 반영합니다. 절차: [`EXPERIMENT_A_LIVE_TUNING.md`](EXPERIMENT_A_LIVE_TUNING.md)

## 검증

```powershell
python scripts/test_fallback_elf.py          # 부팅 OFF, 명령 재사용 거부, lease 만료, 압력 PING 분리, 양쪽 목표 도달과 복귀
python scripts/test_pressure_only_elf.py     # 압력 전용 빌드의 서보 출력 차단
```

실제 빌드된 ELF를 에뮬레이터에서 실행하는 검사입니다. 실물 파형·축 위치·힘·발열 검증은 별도로 필요합니다.
