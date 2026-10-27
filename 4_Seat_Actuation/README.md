[← 전체 구조](../README.md) &nbsp;|&nbsp; ① 센싱 → ② 처리 → ③ RL 제어 → **④ 시트 구동**

# 4. 시트 구동 (Seat Actuation)

> NUCLEO-H723ZG **한 보드**로 볼스터 서보 4개와 압력시트 12채널을 함께 다루는 펌웨어와, 그 보드와 통신하는 PC/Pi 쪽 모듈입니다.
> Raspberry Pi의 RL 런타임은 이 폴더의 보드 통신·차량 입력 검증·CAD 한계 코드를 그대로 import해서 씁니다.

---

## 하드웨어 구성

```text
                    ┌──── D3 / D5 ──► 오른쪽 볼스터 (상부 / 하부 서보)
Raspberry Pi 5 ─USB─┤ NUCLEO-H723ZG
  (or PC)           ├──── D4 / D6 ──► 왼쪽 볼스터 (상부 / 하부 서보)
                    └──── A0~A2 ◄── MUX 3개 × C0~C3 ◄── FSR 압력센서 12개
```

| 기능 | 보드 핀 | MCU 핀 |
|---|---|---|
| 오른쪽 상부 / 하부 서보 | D3 / D5 | PE13 / PE11 (TIM1, 50 Hz) |
| 왼쪽 상부 / 하부 서보 | D4 / D6 | PE14 / PE9 (TIM1, 50 Hz) |
| MUX 주소 S0 / S1 / S2 / S3 | D8 / D9 / D10 / D11 | PF3 / PD15 / PD14 / PB5 |
| MUX 활성 (active low) | D12 | PA6 |
| MUX 신호 0 / 1 / 2 | A0 / A1 / A2 | PA3 / PC0 / PC3_C |

- **서보:** DS5180 270° × 4. 볼스터 하나를 서보 2개가 유니버설 조인트 링크로 움직여 **Yaw(감싸기)**와 **Pitch(기울기)**를 만듭니다.
- **역기구학:** 원하는 구조물 Yaw/Pitch에서 두 크랭크 각도를 계산합니다(C4_R8, 크랭크 반경 24 mm). Python과 C 구현이 같은 결과를 내는지 시험합니다.
- **압력:** 12 bit ADC, 채널당 약 100 Hz. A0 = 좌판, A1 = 측면(볼스터), A2 = 등판.

## 구성

```text
4_Seat_Actuation/
├── stm32/          STM32H723 펌웨어 (C) — 서보 PWM, 압력 스캔, UART 프로토콜, 안전 장치
├── experiment_a/   보드 통신(BoardLink), 차량 입력 검증, 충격 판정, CAD 한계, 기구학, 백업 제어기, 데스크톱 앱
├── fallback/       주행 시뮬레이터 브리지, 규칙 기반 위치 제어 + 압력 감시
└── gui/            수동 제어·자동 측정 스테이션 (각도별 압력·영상 기록)
```

| 폴더 | 최종 시스템에서의 역할 |
|---|---|
| [`stm32/`](stm32/README.md) | 볼스터를 실제로 움직이는 펌웨어. **최종 제어는 `OASIS_FALLBACK v2` 빌드**를 사용 |
| [`experiment_a/`](experiment_a/README.md) | Pi 런타임이 import: 보드 프로토콜, 차량 입력 검증, 충격 판정, CAD 한계. 모델이 없을 때의 **백업 제어기** |
| [`fallback/`](fallback/README.md) | PC에서 주행 시뮬레이터 데이터를 받아 Pi로 넘기는 **브리지**(`beamng_bridge.py`) |
| [`gui/`](gui/README.md) | 기구·압력 특성을 측정할 때 쓴 도구. 각도 조합마다 압력과 영상을 자동 기록 |

## 펌웨어 빌드 종류

같은 소스에서 용도별로 세 가지 이미지를 빌드합니다.

| 빌드 | 옵션 | 용도 | 부팅 시 출력 | 중립 PWM (D3/D4/D5/D6) |
|---|---|---|---|---|
| **OASIS_FALLBACK v2** | `-FallbackControl` | **최종 RL·백업 제어** | PWM OFF | 1800 / 1800 / 1200 / 1200 µs (`FB_ARM` 시) |
| OASIS_CONTROL v6 | (기본) | 측정 스테이션(`gui/`) | 중립 PWM | 1800 / 1200 / 1200 / 1800 µs |
| PRESSURE_ONLY | `-PressureOnly` | 압력 센서 위치 확인 | 서보 출력 차단 | — |

## 안전 장치

| 계층 | 위치 | 내용 |
|---|---|---|
| 세션 ARM | 펌웨어 | 명시적 `FB_ARM`과 새 세션 ID가 있어야만 출력. 재부팅 후 자동 재시작 없음 |
| lease | 펌웨어 | 유효한 명령·PING이 500 ms 동안 없으면 PWM OFF |
| 각도·경로 한계 | 펌웨어 + 호스트 | CAD 간섭 한계(Yaw 22°에서 Pitch 0°)와 튜닝 상한 밖 목표는 거부 |
| 변화율 | 펌웨어 | 서보 펄스를 20 ms마다 축당 최대 2 µs만 변경 |
| watchdog | 펌웨어 | 독립 watchdog 약 2초, 제어 반복이 끝났을 때만 갱신 |
| 프로토콜 | 호스트 | 요청은 한 번에 하나, ACK 200 ms 제한, 명령마다 정확한 응답 확인 |
| 입력 검증 | 호스트 | 200 ms보다 오래된 차량 데이터·순서 오류·충격 의심 → OFF |

> PWM OFF는 신호를 끄는 것입니다. 모터 전원 차단이나 압박 해제를 보장하지는 않습니다.

## 실행 방법

```powershell
# 펌웨어 빌드·업로드 (STM32CubeH7 V1.13.0, CMake, Ninja, STM32CubeProgrammer 필요)
powershell -NoProfile -ExecutionPolicy Bypass -File stm32/scripts/stm32.ps1 -Action Build -FallbackControl
powershell -NoProfile -ExecutionPolicy Bypass -File stm32/scripts/stm32.ps1 -Action List
powershell -NoProfile -ExecutionPolicy Bypass -File stm32/scripts/stm32.ps1 -Action Upload -StLinkSerial '<ST-LINK serial>'

# 주행 시뮬레이터 브리지 (PC)
python -m fallback.beamng_bridge --collector-root C:\path\to\BeamNG_Collector_v0_3
```

`4_Seat_Actuation.code-workspace`를 VS Code로 열면 빌드·업로드·GUI 실행을 작업 목록에서 고를 수 있습니다.

## 검증

```powershell
python -m unittest discover -s fallback/tests -q          # 보드 프로토콜·브리지·제어기
python -m unittest discover -s experiment_a/tests -q      # 보드 링크·차량 검증·충격·기구학·CAD
python -m unittest discover -s gui/tests -q               # 측정 스테이션
python stm32/scripts/test_fallback_elf.py                 # 빌드된 펌웨어 ELF를 에뮬레이터로 실행 (Unicorn 필요)
```

펌웨어 ELF 시험은 실제 빌드 결과물을 Cortex-M7 에뮬레이터에서 돌려 부팅 시 PWM OFF, 명령 재사용 거부, lease 만료, 목표 추종을 확인합니다.

> 원시 측정 데이터, 영상, 빌드 결과물, STM32 SDK는 포함하지 않습니다.
