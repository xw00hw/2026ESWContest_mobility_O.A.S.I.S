[← 전체 구조](../README.md) &nbsp;|&nbsp; ① 센싱 → **② 처리** → ③ RL 제어 → ④ 시트 구동

# 2. 실차 데이터 정제 (Data Processing)

> [`1_Sensor_Acquisition`](../1_Sensor_Acquisition/README.md)이 장치마다 따로 기록한 원시 데이터를 **검증 · 정제 · 동기화**해 하나의 10 Hz 표로 만듭니다.
> 이 결과는 학습 시뮬레이터의 인체·압력 응답 파라미터를 정하는 근거가 됩니다([`3_RL_Control/sim/v3/final/PARAMS_FINAL.md`](../3_RL_Control/sim/v3/final/PARAMS_FINAL.md)).

---

## 파이프라인 한눈에 보기

```text
              ┌─► 1 차량 OBD OCR ───────┐
 00_raw/ ─────┼─► 2 시트 압력 ──────────┼─► 4 시간 동기화 ─► 5 자세 보정 ─► 6 운전석 환산 ─► 01_refined/synced_10hz.csv
              └─► 3 Pi GPS·IMU·카메라 ──┘
```

| 단계 | 폴더 | 입력 | 하는 일 |
|:---:|---|---|---|
| 1 | [`pipeline/1_vehicle_obd_ocr/`](pipeline/1_vehicle_obd_ocr) | 스캐너 엑셀, OBD 앱 화면 녹화 | 화면에서 프레임 추출 → 글리프 템플릿 OCR → 값이 바뀐 시점 복원 → 자체 검사 |
| 2 | [`pipeline/2_seat_pressure/`](pipeline/2_seat_pressure) | 압력 기록기 세션 | 12채널 UART/CSV 검증, 채널별 상태 판정, 스캔 지연·시계 편차 점검 |
| 3 | [`pipeline/3_pi_gps_imu_camera/`](pipeline/3_pi_gps_imu_camera) | Pi 수집기 세션 | NMEA 체크섬·epoch 묶음, IMU 장착 자세 추정, 카메라 프레임 시각, 자세 특징 재계산 |
| 4 | [`pipeline/4_time_sync/`](pipeline/4_time_sync) | 1~3단계 결과 | GPS 기준 시계 + IMU 교차상관으로 장치별 시계 모델 추정, 모든 표에 공통 시각 추가, 10 Hz 통합 표 생성 |
| 5 | [`pipeline/5_pose_correction/`](pipeline/5_pose_correction) | 카메라 영상, 3단계 자세 | RTMPose 재추론 + MediaPipe 비교, 관절 점프·뼈 길이·좌우 뒤바뀜 보정, 관절별 신뢰 표시, 블라인드 감사 |
| 6 | [`pipeline/6_driver_seat_equiv/`](pipeline/6_driver_seat_equiv) | 4단계 통합 표 | 조수석에서 잰 IMU·압력을 운전석 값으로 환산 |

## 단계별 핵심 아이디어

**1 · 차량 신호 OCR** — 상용 OBD 앱은 데이터를 내보내지 않아 화면을 녹화했습니다. 앱 글꼴의 숫자 모양을 템플릿(`templates.npz`)으로 만들어 매 프레임을 읽고,
값이 바뀐 순간만 남겨 시계열(RPM, 차속, 종·횡가속도, 조향각, 요레이트)을 복원합니다.

**4 · 시간 동기화** — 장치마다 시계가 다르고 차 안에서는 네트워크도 없습니다. GPS 시각을 기준 시계로 삼고,
OBD 화면의 속도·요레이트·종·횡가속도를 GPS·IMU 신호와 교차상관해 화면 녹화의 시간 오프셋과 시계 드리프트를 구한 뒤, 모든 센서를 하나의 시간축에 올립니다.

**5 · 자세 보정** — 차 안 카메라 영상은 흔들림·역광·가림이 많습니다. RTMPose로 다시 추론하고 MediaPipe 결과와 비교한 뒤,
갑작스러운 관절 점프, 비현실적인 뼈 길이, 좌우가 뒤바뀐 프레임을 찾아 고치고 관절마다 신뢰도를 표시합니다.

**6 · 운전석 환산** — 안전 때문에 센서를 조수석에 두었습니다. IMU는 강체 운동식 `a_T = a_IMU + α×r + ω×(ω×r)`로 운전석 위치의 가속도로 옮기고,
압력은 채널별 횡·종가속도 응답을 맞춘 뒤 두 좌석의 가속도 차이만큼 보정합니다(관성 성분만 반영).

## 실행 방법

데이터 루트는 이 폴더(`2_Data_Processing/`)입니다. `00_raw/`에 원시 데이터를 두면 `01_refined/`(정제 결과), `02_qc/`(검사 결과), `_work/`(캐시)가 같은 위치에 생깁니다.

```powershell
pip install numpy pandas scipy matplotlib opencv-python openpyxl
$env:OASIS_DRIVE_DATE = "YYYY-MM-DD"        # 수집일 (UTC), 동기화 기준 시각에 사용

python pipeline\1_vehicle_obd_ocr\run.py
python pipeline\2_seat_pressure\run.py
python pipeline\3_pi_gps_imu_camera\run.py
python pipeline\4_time_sync\run.py
python pipeline\5_pose_correction\run.py      # 별도 환경 필요 (mediapipe, rtmlib, onnxruntime-gpu): run.py 상단 참고
python pipeline\6_driver_seat_equiv\run.py
```

| 원시 입력 (`00_raw/`) | 내용 |
|---|---|
| `obd/run1_x431_measurements.xlsx` | 1차 주행: 상용 스캐너 내보내기 |
| `obd/run2_infocar_screenrec.mp4` | 2차 주행: OBD 앱 화면 녹화 |
| `seat/session_*/` | 압력 기록기 세션 ([`pressure_seat`](../1_Sensor_Acquisition/pressure_seat/README.md)) |
| `pi_run1_fragments/session_*/`, `pi_run2/session_run2/` | Pi 수집기 세션 ([`raspberry_pi`](../1_Sensor_Acquisition/raspberry_pi/README.md)) |

## 재현성

- 각 단계는 같은 입력에서 같은 결과를 다시 만들 수 있게 작성했습니다(고정 난수 시드, 단일 스레드 인코딩).
- 스크립트는 본 처리 전에 자체 검사(self-check)를 먼저 실행합니다.
- 원시 데이터는 읽기만 하고 수정하지 않습니다.

> 원시·정제 데이터와 영상, 검사 결과는 저장소에 포함하지 않습니다.
