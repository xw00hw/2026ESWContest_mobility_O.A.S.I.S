# 규칙 기반 제어용 선택 빌드

`OASIS_FALLBACK_CONTROL=ON`은 기존 통합 v6와 별도 펌웨어입니다. 기본값은 OFF이며 `OASIS_PRESSURE_ONLY`와 동시에 선택하면 빌드를 거절합니다. 결과 파일은 `build_fallback/fallback_control_h723.elf`와 `.bin`입니다. `scripts/stm32.ps1 -Action Build -FallbackControl`로 선택할 수 있고 PC에 맞는 CMake/Ninja/GCC/SDK 경로를 인자로 전달합니다. 이 구현 작업에서는 실제 업로드·UART·모터 시험을 수행하지 않습니다.

## 출력과 보정 계약

- 첫 애플리케이션 동작은 TIM1을 끄고 D3/D4/D5/D6를 GPIO LOW로 만드는 것입니다. 리셋부터 이 코드 실행 전까지의 핀 상태를 소프트웨어로 보장한다는 뜻은 아닙니다.
- D3/D4/D5/D6 = PE13/PE14/PE11/PE9 = TIM1 CH3/CH4/CH2/CH1, AF1, 50Hz입니다. MUX D8~D11, EN D12, ADC A0/A1/A2는 유지합니다.
- 현재 장착에서 명시적 `FB_ARM`은 2026-10-28 확정 영점 D3/D4/D5/D6 = **1800/1800/1200/1200µs**를 출력합니다. 이 최초 출력은 2µs 변화 제한의 예외이며 실제 축이 움직일 수 있습니다. 호스트는 장치·장착·보정 프로필 일치와 사용자의 초기 위치 적용 승인을 먼저 확인해야 합니다. 기존 수집 v6의 영점과 과거 기록은 변경하지 않습니다.
- 환산은 기존 `joint_math.c`의 C4_R8, DS5180SSG 270°, 2000µs/270°를 재사용합니다. 이 이미지의 고정 상한은 구조물 yaw 0~27°, pitch ±8°, 펄스 500~2500µs와 IK 도달 조건입니다. 호스트의 실측 영역 제한을 대신하지 않습니다.
- 최초 ARM의 영점 펄스는 비어 있는 타이머 프레임 뒤 자연 overflow에서 네 채널 동시에 시작합니다. 이후에는 자연 프레임당 최대 1회, 모든 허용 펄스가 끝난 CNT 3000~17000µs 구간에 CCR를 준비합니다. UDIS로 네 preload를 묶으며 동작 중 UG를 발생시키지 않습니다.
- 목표를 바꿔도 20ms 추종 시계는 다시 시작하지 않습니다. 각 변경은 이전 명령 대비 최대 2µs/축이며 늦은 반복에서 밀린 변경을 몰아서 수행하지 않습니다. CPU 부하·USB 지연·실제 모터 응답시간은 실측하지 않았습니다.
- `FB_OFF`와 fault는 즉시 PWM을 중단하고 GPIO LOW로 전환합니다. 진행 중 펄스를 자를 수 있으며 자동 중립 복귀를 하지 않습니다. 외부 전원을 차단하는 기능은 아닙니다.

## ASCII 프로토콜

460800bps, 8N1, LF 종결. 정수 필드에는 공백, `+`, 지수, 실수, 빈 필드가 허용되지 않습니다. 양의 정수 ID 범위는 1~4294967295이며 음수는 pitch에만 허용됩니다.

아래 v1 예시는 과거 프로토콜·보정 기록입니다. 현재 v2는 `neutral_us=1800:1800:1200:1200`을 보고하고 `FB_ARM,<session_id>,<tuning_revision>`을 사용합니다. 다른 영점의 HELLO는 현재 호스트에서 거부합니다.

```text
CONTROL_HELLO
#HELLO,OASIS_FALLBACK,v1,yaw_min_mdeg=0,yaw_limit_mdeg=27000,pitch_limit_mdeg=8000,servo_range_deg=270,neutral_us=1800:1200:1200:1800,boot_pwm=OFF,lease_ms=500,tick_ms=20,max_step_us=2,ds_outputs=4,ik_revision=8,session_floor=0

FB_ARM,<session_id>
#ACK,FB_ARM,<session_id>

FB_SET,<session_id>,<command_id>,<left_yaw_mdeg>,<left_pitch_mdeg>,<right_yaw_mdeg>,<right_pitch_mdeg>
#ACK,FB_SET,<session_id>,<command_id>,<left_yaw_mdeg>,<left_pitch_mdeg>,<right_yaw_mdeg>,<right_pitch_mdeg>

FB_PING,<session_id>,<counter>
#ACK,FB_PING,<session_id>,<counter>

FB_OFF
#ACK,FB_OFF
```

`session_floor`는 부팅 후 마지막으로 수락한 세션 ID입니다. 새 ARM에는 반드시 이보다 큰 ID가 필요합니다. 현재 세션을 그대로 재ARM하거나 예전 세션을 재사용할 수 없습니다. ARM 중 ARM도 거절하고 출력을 끕니다. 재부팅하면 floor는 0이 되므로 호스트는 MCU 재부팅을 감지하고 자동 재ARM하지 않아야 합니다. ARM 도중 하드웨어 오류가 나도 그 ID는 소비됩니다.

SET ID와 PING counter는 각각 세션 안에서 엄격히 증가해야 합니다. 0에서 시작하므로 첫 요청은 1 이상입니다. ACK는 수락 사실이며 실제 위치 도달 확인이 아닙니다. SET은 최신 목표만 바꾸고 누적 이동이나 반복 왕복을 만들지 않습니다. ID 재사용·잘못된 세션·잘못된 입력은 오류이며, ARM 상태라면 출력도 끄고 새 세션을 요구합니다.

오류 응답은 `#ERR,<command_id 또는 0>,<reason>`입니다. reason은 `INVALID`, `DISARMED`, `SESSION`, `SESSION_REPLAY`, `SET_REPLAY`, `PING_REPLAY`, `ALREADY_ARMED`, `MOTOR_BOUNDS`, `HARDWARE`, `WATCHDOG`, `UNSUPPORTED`입니다. 일반 `ARM`, `SET`, `HOLD`는 호환 명령으로 처리하지 않습니다.

## 상태·유효시간

`STATE` 응답과 명목 5Hz 상태는 기존과 같은 18필드입니다.

```text
#STATE,mcu_ms,armed,moving,LY,LP,RY,RP,target_LY,target_LP,target_RY,target_RP,D3_us,D4_us,D5_us,D6_us,command_id,fault
```

각도는 mdeg, 펄스는 µs입니다. boot에서는 모든 수치가 0입니다. OFF 후에는 마지막 명령 자세가 남지만 펄스는 모두 0이며 target은 마지막 명령 자세로 정리됩니다. 따라서 기존 v6의 "모든 상태에서 자세와 PWM 일치" 파서를 그대로 쓸 수 없습니다. 실제 위치를 측정한 값은 없습니다.

유효한 FB_ARM/FB_SET/FB_PING만 motor lease를 갱신합니다. 마지막 갱신 후 500ms 이상임을 다음 검사에서 확인하면 `LEASE` fault로 GPIO LOW로 전환합니다. 500ms는 검사 기준이며 정확한 500.000ms 하드웨어 차단 보장은 아닙니다. 송신 대기 중·블로킹 송신 전후·ADC 뱅크 사이·명령 처리 전후에 검사하지만 한 HAL 호출의 블로킹 시간(UART timeout 20ms, ADC conversion timeout 5ms)과 실행 지연이 더해질 수 있습니다. 실제 최대 지연은 아직 측정하지 않았습니다. 일반 압력 `PING`은 motor lease를 갱신하지 않습니다. FSR의 3초 스캔 watchdog·ADC/UART 오류 경로는 유지하며 해당 오류도 fallback 출력 OFF를 유발합니다. `DISABLE`은 압력 스캔만 종료하므로 전체 종료에는 FB_OFF와 DISABLE을 각각 확인합니다.

부팅 `BOOT`, 정상 ARM `NONE`, 정상 OFF `OFF` 외에 `LEASE`, `PROTOCOL`, `PATH_BOUNDS`, `RATE_BOUNDS`, `HARDWARE`, `WATCHDOG`, FSR가 제공하는 `FSR_FAULT`/`UART_FAULT` 등이 보고됩니다. FB_OFF는 기존 fault를 지우지 않습니다. 새 세션 ARM을 명시적으로 수락할 때만 해제됩니다.

fallback 빌드에서 RX는 poll당 최대 32바이트·완성 행 1개로 제한하며 dispatch 전에 만료를 확인합니다. PWM 갱신 전 register readback을 검증하여 손상된 CCR를 새 값으로 덮어 숨기지 않습니다. IWDG1은 LSI /64, reload999(32kHz 가정 명목 2초)이며 FSR·제어의 한 반복이 끝난 경우에만 feed합니다. TIM1/IWDG debug freeze를 해제합니다. CPU halt 시 즉각 OFF는 아니며 watchdog reset까지 기존 펄스가 남을 수 있습니다. ARM 상태의 디버거 halt/step는 정상 운용 방식이 아닙니다.

## 검증

`scripts/test_fallback_elf.py`는 실제 생성 ELF를 Unicorn으로 실행합니다. Cortex-M7 시험은 실제 출력·프로토콜·IWDG register 코드를 사용하고, double-precision 명령 미지원 때문에 수학/연속 추종 시험은 ARM_MAX 모델과 PWM 경계 stub을 사용합니다. UDIS 중 CCR마다 강제 rollover, 부팅 OFF, 명령 재사용, 5ms 목표 스트림, lease 만료, 압력 PING 분리, RX 제한, 양쪽 목표 도달과 복귀를 검사합니다. 실물 파형·축 위치·힘·간섭·발열·시간 지연 검증은 별도입니다.
