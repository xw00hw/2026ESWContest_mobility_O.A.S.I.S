[← 1. 센서 데이터 수집](../README.md) &nbsp;|&nbsp; **① 센싱** → ② 처리 → ③ RL 제어 → ④ 시트 구동

# 시트 압력 주행 기록기 (pressure_seat)

> FSR 압력센서 12개가 달린 시트의 값을 주행 내내 CSV로 기록합니다.
> Raspberry Pi 수집기와 **같은 폴더 구조·같은 시각 열**을 써서 나중에 바로 합칠 수 있습니다.

---

## 하드웨어 구성

```text
FSR 12개 ──► MUX 3개 (각 C0~C3) ──► NUCLEO-H723ZG ADC A0/A1/A2 ──► ST-LINK USB (460800 bps) ──► 노트북
```

| 항목 | 값 |
|---|---|
| 보드 | NUCLEO-H723ZG |
| 센서 배치 | A0 = 좌판 · A1 = 측면(볼스터) · A2 = 등판, 각 MUX 채널 C0~C3 |
| 샘플링 | 센서당 100 Hz |
| ADC | 12 bit (0 ~ 4095), 공칭 기준 전압 3.3 V |

## 실행 방법

```powershell
py -m pip install -r requirements.txt          # pyserial
py collect_seat.py                             # ST-LINK 포트 자동 탐색, Ctrl+C로 정지
py collect_seat.py --port COM7 --suffix run1   # 포트·세션 이름 지정
py collect_seat.py --list-ports
```

- 1초마다 프레임 수, 손실 패킷, ADC 오류, 최신 12채널 값을 한 줄로 표시합니다.
- USB가 빠지거나 보드가 리셋되거나 3초간 데이터가 끊기면, 오류를 기록하고 **같은 세션으로 다시 연결**합니다.
- 다른 프로그램이 COM 포트를 쓰고 있으면 실행할 수 없습니다.

## 출력

```text
data/sessions/YYYY-MM-DD/session_YYYYMMDD_HHMMSS[_suffix]/
├── seat/pressure.csv       12개 센서 한 번 스캔 = 한 행 (공칭 100 Hz)
├── seat/raw_uart.log       수신한 모든 줄: "<monotonic_ns> <wall_ns> <line>"
├── events.jsonl            기록 시작, 연결 오류 등 사건
├── metadata.json           시작 시 시계·NTP 오프셋, 포트, 펌웨어, 설정 사본
├── session_summary.json    카운터, 실측 주기, 종료 시 NTP 오프셋
└── INCOMPLETE              기록 중이거나 비정상 종료 시에만 존재
```

### `seat/pressure.csv` 열

| 열 | 설명 |
|---|---|
| `host_monotonic_ns`, `host_wall_time_ns` | 프레임 첫 패킷의 노트북 수신 시각 |
| `sequence_id` | 세션 안 프레임 번호 |
| `packet_seq`, `mcu_t_ms` | 펌웨어 패킷 번호와 MCU 시각(ms) |
| `packets_in_frame` | MUX 채널 4개가 모두 오면 4 |
| `frame_valid` | 모든 패킷이 오고 모든 ADC 읽기가 성공했을 때만 1 |
| `<센서>_raw` × 12 | ADC 값 0~4095. ADC 오류·패킷 손실이면 0이 아니라 **빈 칸** |
| `<센서>_v` × 12 | `raw × vref / 4095` (V) |

## 값을 해석할 때 주의할 점

- **전압이지 압력이 아닙니다.** N · kg · kPa 보정이나 기준선 차감은 하지 않습니다.
- **한 행은 한 순간이 아닙니다.** MUX 채널 4개를 약 2.5 ms 간격으로 차례로 스캔하므로 한 행은 최대 약 10 ms에 걸칩니다.
- **시각은 USB 수신 시각**이라 수 ms의 흔들림이 있습니다. 더 정밀한 분석을 위해 `mcu_t_ms`와 원본 로그를 함께 남깁니다.
- 센서의 좌·우 위치는 이 기록기에 저장하지 않습니다. 실차 데이터에서 각 채널이 횡가속도에 어떻게 반응하는지로 좌·우를 추정했습니다.

## 검증

```powershell
py -m pip install pytest
py -m pytest tests -q        # 가짜 보드 사용, 시리얼 포트를 열지 않음
```

연결 확인·하트비트, 프레임 조립, ADC 오류 시 빈 칸 처리, 패킷 손실 계산, 보드 리셋 후 재연결, 정상 종료를 검사합니다.
