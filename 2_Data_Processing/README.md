# 2_Data_Processing — 실차 주행 데이터 정제 파이프라인

`1_Sensor_Acquisition`이 각 장치에서 따로 기록한 원시 데이터를 검증·정제하고 **하나의 시간축**으로 합칩니다.
결과는 시뮬레이터의 인체·압력 응답 파라미터를 정하는 근거가 됩니다(`3_RL_Control/sim/v3/final/`).

```text
00_raw/ ──► 1 차량 OBD OCR ──┐
        ──► 2 압력시트 ──────┼──► 4 시간 동기화 ──► 5 자세 보정 ──► 6 운전석 환산 ──► 01_refined/run{1,2}/synced_10hz.csv
        ──► 3 Pi GPS·IMU·카메라 ┘
```

| 단계 | 폴더 | 하는 일 |
|---|---|---|
| 1 | `pipeline/1_vehicle_obd_ocr/` | 1차: 상용 스캐너 엑셀 변환. 2차: OBD 앱 화면 녹화를 프레임 추출 → 글리프 템플릿 OCR(`templates.npz`) → 값 변화 시점 복원 → 자체 검사 |
| 2 | `pipeline/2_seat_pressure/` | 압력 12채널 UART/CSV 검증, 채널별 상태 판정, 스캔 지연·벽시계 편차 점검 |
| 3 | `pipeline/3_pi_gps_imu_camera/` | NMEA 체크섬·epoch 묶음, IMU 장착 자세 추정, 카메라 프레임 시각, 실시간 pose 특징 재계산 |
| 4 | `pipeline/4_time_sync/` | GPS 기준 시계 + IMU 교차상관으로 장치별 시계 모델 추정, 모든 표에 공통 시각 추가, 10 Hz 통합 표 생성 |
| 5 | `pipeline/5_pose_correction/` | RTMPose 재추론 + MediaPipe 비교, 관절 점프·뼈 길이·좌우 뒤바뀜 보정, 관절별 신뢰 표시, 블라인드 감사 |
| 6 | `pipeline/6_driver_seat_equiv/` | 조수석에서 잰 IMU·압력을 강체 운동식과 채널별 가속 응답으로 운전석 값으로 환산 |

## 실행

데이터 루트는 이 폴더(`2_Data_Processing/`)입니다. 원시 데이터를 `00_raw/`에 두면 `01_refined/`(정제 결과),
`02_qc/`(검사 결과), `_work/`(캐시)가 같은 위치에 생깁니다. 원시·정제 데이터와 영상은 저장소에 포함하지 않습니다.

```powershell
pip install numpy pandas scipy matplotlib opencv-python openpyxl
$env:OASIS_DRIVE_DATE = "YYYY-MM-DD"        # 수집일 (UTC), 동기화 기준 시각에 사용

python pipeline\1_vehicle_obd_ocr\run.py
python pipeline\2_seat_pressure\run.py
python pipeline\3_pi_gps_imu_camera\run.py
python pipeline\4_time_sync\run.py
python pipeline\5_pose_correction\run.py      # 별도 환경: 5_pose_correction/run.py 상단 참고 (mediapipe, rtmlib, onnxruntime-gpu)
python pipeline\6_driver_seat_equiv\run.py
```

| 원시 입력 (`00_raw/`) | 내용 |
|---|---|
| `obd/run1_x431_measurements.xlsx`, `obd/run2_infocar_screenrec.mp4` | 1차 스캐너 내보내기, 2차 OBD 앱 화면 녹화 |
| `seat/session_*/` | 압력 기록기 세션 (`1_Sensor_Acquisition/pressure_seat`) |
| `pi_run1_fragments/session_*/`, `pi_run2/session_run2/` | Pi DAQ 세션 (`1_Sensor_Acquisition/raspberry_pi`) |

각 단계는 같은 입력에서 같은 결과를 다시 만들 수 있도록 작성했고, 스크립트마다 자체 검사(self-check)를 먼저 실행합니다.
