[← 전체 구조](../README.md) &nbsp;|&nbsp; **① 센싱** → ② 처리 → ③ RL 제어 → ④ 시트 구동

# 1. 센서 데이터 수집 (Sensor Acquisition)

> 실제 차량을 주행하며 **차량 거동 · 시트 압력 · 운전자 자세**를 동시에 기록하는 수집기입니다.
> 여기서 모은 데이터는 [`2_Data_Processing`](../2_Data_Processing/README.md)에서 정제되어, 학습 시뮬레이터의 인체·압력 응답을 정하는 근거가 됩니다.

---

## 이 폴더가 하는 일

세 종류의 장치가 **각자 따로, 동시에** 기록하고, 기록이 끝난 뒤 공통 UTC 시간축으로 합칩니다.

```text
 GPS · IMU · 카메라 ──► Raspberry Pi 5 (oasis_daq) ──┐
                                                      ├──► 장치별 세션 폴더 ──► align_sessions.py ──► 하나의 CSV
 시트 압력 12채널 ───► NUCLEO-H723ZG ──► 노트북 ─────┤
                                                      │
 차량 OBD ──────────► 상용 스캐너 / OBD 앱 화면 ──────┘   (화면 녹화는 2_Data_Processing에서 OCR)
```

| 기록 장치 | 하드웨어 | 실행 위치 | 코드 | 출력 |
|---|---|---|---|---|
| GPS · IMU · 카메라 · 자세 | Raspberry Pi 5 | Pi | [`raspberry_pi/`](raspberry_pi/README.md) | `gps/`, `imu/`, `camera/`, `pose/` |
| 시트 압력 (FSR 12개) | NUCLEO-H723ZG | 노트북 (USB) | [`pressure_seat/`](pressure_seat/README.md) | `seat/pressure.csv` |
| 차량 데이터 | 상용 OBD 스캐너 · 앱 | 스캐너 · 휴대폰 | [`../2_Data_Processing/`](../2_Data_Processing/README.md) 1단계 | `obd/` |

## 구성

```text
1_Sensor_Acquisition/
├── raspberry_pi/        GPS + IMU + A필러 카메라 + MediaPipe 자세 수집 (oasis_daq 패키지)
├── pressure_seat/       압력시트 12채널 주행 기록기
└── align_sessions.py    장치별 세션을 UTC 기준 하나의 표로 정렬
```

## 공통 기록 형식

모든 기록기는 같은 구조의 세션 폴더를 만들고, 모든 행은 같은 두 시각 열로 시작합니다.

```text
<data root>/sessions/YYYY-MM-DD/session_YYYYMMDD_HHMMSS[_suffix]/
├── metadata.json          시작 시 시계 정보, 장치, 설정
├── session_summary.json   종료 시 통계
├── INCOMPLETE             정상 종료 시에만 삭제 (비정상 종료 감지용)
└── <센서>/*.csv
```

| 열 | 의미 | 용도 |
|---|---|---|
| `host_monotonic_ns` | 기록 장치 자체의 단조 시계 | **같은 장치 안**의 순서·간격 |
| `host_wall_time_ns` | 그 장치의 UTC 시각 (ns) | **장치 사이**의 연결 |

## 시간 동기화 원리

장치 사이에 연결선이 없으므로 공통 기준은 UTC입니다.

1. **장치 내부:** 세션마다 `벽시계 − 단조시계`의 중앙값 하나로 단조 시계를 UTC로 옮깁니다. 단조 시계의 매끄러운 간격은 유지하고, 벽시계가 중간에 튀는 것은 무시합니다.
2. **장치 간 오차 제거:**
   - 노트북: 세션 시작·끝에 NTP 서버와의 차이를 재서 저장합니다(휴대폰 핫스팟 사용).
   - Pi: 차 안에서 네트워크가 없으면 `gps/gps.csv`의 GPS 시각으로 시계를 보정합니다.
3. **정렬:** `align_sessions.py`가 각 스트림의 UTC 범위와 적용한 오프셋을 출력하고, 지정 주기(기본 20 Hz)의 격자마다 각 스트림의 **가장 최근 값**을 붙입니다(보간 없음). 스트림마다 `<stream>_age_ms` 열로 값의 나이를 남깁니다.

예상 정확도는 **수십 ms**입니다. NTP와 USB 수신 지연이 한계이며, 그보다 정밀하려면 하드웨어 트리거가 필요합니다.
실차 데이터의 최종 정렬은 더 정밀한 방식(GPS 기준 시계 + IMU 교차상관)으로 [`2_Data_Processing`](../2_Data_Processing/README.md) 4단계에서 다시 수행합니다.

## 실행 방법

```powershell
# 1. 노트북과 Pi를 같은 휴대폰 핫스팟에 연결 (두 시계 모두 NTP 동기화)
# 2. Pi 수집 시작 (raspberry_pi/docs/OPERATIONS.md 참고)
# 3. 노트북에서 압력 기록 시작
py 1_Sensor_Acquisition\pressure_seat\collect_seat.py --suffix run1
# 4. 주행 후 Ctrl+C로 정지, Pi 세션 폴더를 노트북으로 복사
# 5. 정렬
py 1_Sensor_Acquisition\align_sessions.py --pi <pi 세션> --seat <압력 세션> --out aligned.csv
py 1_Sensor_Acquisition\align_sessions.py --selftest       # 자체 검사
```

원본 세션 폴더는 절대 수정하지 않으므로, 옵션을 바꿔 몇 번이든 다시 실행할 수 있습니다.

## 실차 수집 시 유의 사항

- 안전 때문에 IMU와 압력시트는 **조수석**에서 측정했습니다. 운전석 값은 [`2_Data_Processing`](../2_Data_Processing/README.md) 6단계에서 강체 운동식으로 환산합니다.
- 차량 OBD 값은 상용 스캐너와 OBD 앱 화면 녹화로 받았고, 화면 녹화는 OCR로 시계열을 복원했습니다.
- 주행마다 각 센서와 착석자가 어느 좌석에 있었는지, IMU와 좌석 사이 거리를 기록해야 합니다.
