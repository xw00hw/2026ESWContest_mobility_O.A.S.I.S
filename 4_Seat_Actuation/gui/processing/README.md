# 2026-10-25 벤치 데이터 정리

완료된 실측 세션 3개를 입력으로 사용한다. 원본은 읽기만 하며 모델 추론은 로컬 CPU에서 수행한다.

| 세션 | 시험 | 자세 수 |
|---|---|---:|
| `20261025_003530_9c4e477c` | 양쪽 동시 Yaw 0…20°, Pitch −8…8° | 357 |
| `20261025_005459_3512cd6e` | 왼쪽 Yaw 0…20°, Pitch −8…8°, 오른쪽 고정 | 357 |
| `20261025_013054_fdbdd0b4` | 좌·우 독립 Yaw 0…27°, Pitch 양쪽 0° | 784 |

기본 출력은 `data/processed/bench_20261025_pose_v1/`이다. 모델과 데이터는 Git에 넣지 않는다.
기구 왼쪽은 D4·D6, 오른쪽은 D3·D5다. 원래 모터 매핑과 PWM은 각 세션의 `source_metadata.json`에 보존한다.
`all_points.csv`는 1,498개 측정 구간의 압력·자세 추정치를 한 행씩 묶는다.
`analysis_points.csv`는 좌우 각도, 압력 12채널 평균, 품질 검사 후 2D 포즈 평균만 추린 간단 표다.
`sessions.csv`, 세션별 `quality_report.json`, `source_manifest.json`에서 누락·유효율·출처를 확인한다.

## 실행

Python 3.13 가상환경에 `requirements.txt`를 설치한다. GUI용 환경은 변경하지 않는다.
공식 [RTMPose 모델 목록](https://github.com/Tau-J/rtmlib)의
`rtmpose-m_simcc-body7_pt-body7_420e-256x192-e48f03d0_20230504.zip`을 내려받아 ONNX를 꺼낸다.
[모델 원본](https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-m_simcc-body7_pt-body7_420e-256x192-e48f03d0_20230504.zip).
모델은 RTMLib의 전처리·SimCC 디코더와 OpenVINO CPU / ONNX Runtime DirectML GPU로 실행한다.
샘플 6개에서 CPU·GPU의 디코딩된 좌표가 동일함을 확인했다.
행별 `inference_backend`와 런타임 기록에서 실행 backend를 확인한다.
DirectML은 별도 환경에 `rtmlib`를 먼저 설치한 뒤 `onnxruntime-directml==1.24.4`를 설치한다.

```powershell
python prepare_bench.py --input <sessions> --output <processed> --model <rtmpose_m.onnx>
python organize_bench.py --input <sessions> --output <processed> --stage all
python report_bench.py --input <sessions> --output <processed> --model <rtmpose_m.onnx>
python validate_outputs.py --output <processed>
python -m unittest discover -s . -p test_processing.py
```

명령은 이 디렉터리 기준이다. OpenCV의 비ASCII 경로 처리 문제가 있는 환경에서는
`C:/Temp/esw_bench_inputs`, `C:/Temp/esw_bench_outputs` 같은 디렉터리 junction을 사용한다.
각 세션을 `--session <id>`로 개별 실행할 수 있다. 이미 완료된 포즈/센서 작업은 건너뛴다.
MJPG 영상에는 `--decoder jpeg`로 미선택 프레임의 JPEG 디코딩을 생략할 수 있다.
`image_decoder`에 FFmpeg/libjpeg를 구별한다. 3개 세션 각 2프레임 비교에서
상체 관절 차이는 최대 2.86px, 평균 1px 미만이며 유효성 판정은 같았다.
원본 영상과 실제 선택 프레임 번호는 바뀌지 않는다. 비교 결과를 데이터 루트 `validation/`에 보존한다.
포즈 작업은 1,000프레임마다 checkpoint를 남기며 중단 후 재실행할 때 기존 추론값을 재사용한다.
모델·크롭·관절 유효성·샘플링 설정을 바꾸면 새로운 출력 디렉터리를 사용한다.
검증된 실행 backend/decoder 전환은 행별 실행 정보를 남겨 기존 checkpoint와 함께 사용할 수 있다.

## 시간·좌표·관측 범위

- 기준 시각: PC `perf_counter_ns`/Windows QPC의 `host_monotonic_ns`(정수 ns).
  `session_s`는 세션 origin에서 경과한 초다. 실제 USB·카메라 지연은 측정되지 않았다.
- MCU 시계는 PC보다 약 0.45~0.48% 빠른 실측이 있다. 원래 `mcu_ms`를 영상 초와 직접 결합하지 않는다.
- 영상은 30fps CFR이고 일부 프레임은 반복이다. 매 세 번째 표시 프레임을 선택해 10Hz로 추정한다.
  동일 `capture_index`는 한 번만 추론한다. 포즈는 실제 캡처/수신 `host_monotonic_ns`에 붙인다.
  `video_time_s`·`overlay_time_s`는 표시 시간이며 실제 센서 시각과 구별한다.
- ROI는 원본 1280×720의 `(300,120)–(920,720)`이다. 배경 사람 대신 중앙 착석자를 대상으로 한다.
  저장 좌표는 크롭 기준이 아닌 **원본 영상 기준 pixel**, +x 오른쪽/+y 아래다.
  `left_*`/`right_*` 관절명은 COCO의 피사체 해부학적 좌우이며 GUI의 기구 좌우와 별개다.
- COCO 17개 관절의 x/y/score와 별도 `*_valid`를 저장한다. score ≥0.5, ROI 내부,
  코·양어깨의 착석자 위치 조건을 만족해야 유효하다. 스무딩·결측 보간은 하지 않는다.
  무릎·발목은 화면 밖이므로 모델 좌표가 있어도 항상 invalid다. 골반은 가려질 수 있어 유효율을 별도로 확인한다.
- `shoulder_tilt_deg`: 화면 왼쪽 어깨에서 오른쪽 어깨로 잇는 선의 기울기, +는 오른쪽이 아래.
  `torso_lean_deg`: 양어깨 중심과 양골반 중심으로 계산한 2D 기울기, +는 상체가 화면 오른쪽.
  `head_offset_shoulder_ratio`: 코와 어깨 중심의 수평 차/어깨 간 거리, +는 화면 오른쪽.
  골반 또는 어깨가 무효면 해당 feature는 NaN이다. 신체의 3D Yaw/Pitch 추정값이 아니다.
- 왼쪽 시험의 영상 239–280초는 카메라 재조준·카메라 쪽 손 뻗기를 육안 확인하여 보수적으로 표시한 구간이다.
  `camera_reposition_flag`와 `manual_quality_flags.csv`에 남긴다. `pose_review_required`는 이 구간,
  착석자 gate 실패, 유효 상체 관절 9개 미만 중 하나다. 원래 좌표/feature는 유지하고,
  `pose_analysis_valid`가 참인 표본으로 `*_analysis_mean`/`*_analysis_n`을 추가 집계한다.
  빈 구간을 보간하지 않는다. 카메라 안정성이 다른 구간까지 검증됐다는 뜻은 아니다.
  조정 전/중/후는 `camera_epoch=0/1/2`이며, 자세별 표에 epoch 최솟값/최댓값을 남긴다.
  전후 좌표계는 재보정하지 않았으므로 영상 기울기·크기를 같은 카메라 설정으로 간주하여 비교하지 않는다.
- 압력은 12bit ADC와 공칭 Vref=3.3V 기준 전압이다. 힘·압력 보정 및 센서 공간 배치는 미제공이다.
  `A0_C0` 등 전기적 ID를 유지하며 CoP나 N/kPa로 바꾸지 않는다.
- 기구 Yaw/Pitch·PWM은 명령/펌웨어 내부 상태다. 실측 축각이나 실제 인체 자세와 동일하지 않다.

## 산출물

| 세션별 파일 | 행 기준 / 용도 |
|---|---|
| `points.csv`, `points.parquet` | 정확한 2초 dwell 구간 하나당 한 행; 명령각·PWM·압력 12채널 통계·포즈 통계 |
| `point_pressure.csv` | dwell × 센서; 유효 표본 수·평균/표준편차/최소/최대·0값/포화 비율 |
| `pressure_long.parquet` | 원시 센서 값 하나당 한 행, 원래 시각과 dwell ID 추가 |
| `motion.parquet` | 원시 상태 보고 하나당 한 행 |
| `pose_samples.parquet` | 10Hz 표시 샘플, 원본 프레임·캡처 ID·시각·17개 관절·유효성·2D features |
| `pose_unique.parquet` | 중복 캡처를 제거한 추정값; dwell 통계는 이 표로 계산 |
| `frame_index.parquet` | 전체 원본 CFR 프레임과 실제 캡처 시각·반복 여부 |
| `frame_pose_index.parquet` | 전체 원본 프레임 → 가장 가까운 추정 캡처 ID/시각, 허용 ±100ms |
| `pressure_10hz.parquet` | origin 기준 100ms 구간 내 유효 압력 평균·population std·범위·개수 |
| `aligned_10hz.parquet` | 위 구간 중심 시각에 포즈 nearest ±100ms, 상태 backward 최대 500ms 결합 |
| `pose_overlay.mp4` | 620×600 중앙 크롭, 10fps, 유효 관절 스켈레톤과 표시/캡처 시간 |
| `sensor_quality.csv`, `quality_report.json` | 오류·시계 드리프트·포즈 결측·영상 길이 검사 |
| `source_manifest.json` | 입력 파일 크기·SHA-256; 원본 위치 참조 |
| `validation.json` | 표 재읽기·좌표/시각 범위 검사와 출력 영상 전체 디코딩 결과 |

`aligned_10hz`는 오프라인 nearest join이므로 다음 포즈 샘플을 고를 수도 있다.
`pose_time_delta_ms`에 부호 있는 시간차, `pose_age_ms`에 절댓값을 저장한다.
허용 시간 밖의 값은 결측으로 남긴다. 실시간 예측기의 인과적 입력으로 바로 사용하지 않는다.
압력 bin은 dwell 경계를 걸칠 수 있다. 자세별 비교에는 원래 host 시각으로
`[dwell_start_ns,dwell_end_ns)`에 포함된 표본만 집계한 `points`를 사용한다.
`dwell_point_index=-1`은 이동·안정·시작/종료 등 측정 구간 밖이다.
각도는 `left_yaw_deg`, `left_pitch_deg`, `right_yaw_deg`, `right_pitch_deg`를 사용한다.
원본 호환용 `yaw_deg`·`pitch_deg`만으로 좌우 독립 시험을 표현하지 않는다.
낮은 신뢰도와 0V에 가까운 채널은 삭제하지 않고 유효성·통계로 남긴다.

본 데이터는 단일 수집의 관찰값이다. 조합 간 차이에 인체 움직임·시간 경과·하중 변화가 섞일 수 있다.
표준편차는 구간 안 표본의 분산이며 반복 실험 간 분산이 아니다.
