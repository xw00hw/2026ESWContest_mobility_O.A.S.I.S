[← 볼스터 측정 스테이션](../README.md) &nbsp;|&nbsp; ① 센싱 → **② 처리** → ③ RL 제어 → ④ 시트 구동

# 벤치 측정 데이터 후처리 (gui/processing)

> 측정 스테이션으로 기록한 AUTO 세션(볼스터 각도 조합 × 압력 12채널 × 착석자 영상)을
> **자세별 압력 통계 + 착석자 2D 골격**이 붙은 분석용 표로 정리합니다.

---

## 입력과 출력

| 입력 세션 | 시험 | 자세 수 |
|---|---|---:|
| 양쪽 동시 | Yaw 0 ~ 20°, Pitch −8 ~ 8° | 357 |
| 왼쪽 단독 | Yaw 0 ~ 20°, Pitch −8 ~ 8°, 오른쪽 고정 | 357 |
| 좌·우 독립 | Yaw 0 ~ 27° × 0 ~ 27°, Pitch 0° | 784 |

결과: 측정 구간 1,498개 각각에 대해 명령 각도·PWM·압력 12채널 통계·자세 통계를 한 행으로 묶은 표(`all_points.csv`, `analysis_points.csv`)와 세션별 상세 표.

## 처리 과정

```text
원본 세션 (읽기 전용)
  ├─ 압력·명령 상태 ──────────────► 측정 구간별 통계 (평균·표준편차·최소·최대·유효 수)
  └─ 영상 30 fps ─► 10 Hz 샘플링 ─► RTMPose (중앙 착석자 ROI) ─► COCO 17관절 + 유효성 ─► 2D 자세 특징
                                                                                       │
                                     100 ms 구간 기준 정렬 (압력 평균 + 최근접 자세 ±100 ms) ◄┘
```

- **시각:** PC `perf_counter_ns`가 기준입니다. MCU 시계는 PC보다 약 0.45 ~ 0.48 % 빨라 MCU 시각과 영상 시각을 직접 결합하지 않습니다.
- **착석자 선택:** 원본 1280×720의 중앙 ROI만 사용해 배경 사람을 제외합니다. 좌표는 원본 영상 기준 픽셀로 저장합니다.
- **유효성:** 점수 ≥ 0.5, ROI 내부, 코·양어깨 위치 조건을 만족한 관절만 유효로 표시합니다. 스무딩·보간은 하지 않습니다.
- **2D 특징:** 어깨선 기울기, 어깨 중심 – 골반 중심 기울기, 머리 수평 오프셋 / 어깨 폭. 3D 자세 추정값은 아닙니다.
- **품질 표시:** 카메라 재조준 구간 등 사람이 확인한 문제 구간은 지우지 않고 플래그로 남깁니다.

## 실행 방법

```powershell
pip install -r requirements.txt     # 별도 Python 환경 권장
# RTMPose-m ONNX: rtmpose-m_simcc-body7_pt-body7_420e-256x192 (rtmlib 공식 모델)

python prepare_bench.py   --input <sessions> --output <processed> --model <rtmpose_m.onnx>
python organize_bench.py  --input <sessions> --output <processed> --stage all
python report_bench.py    --input <sessions> --output <processed> --model <rtmpose_m.onnx>
python validate_outputs.py --output <processed>
python -m unittest discover -s . -p test_processing.py
```

- 추론은 OpenVINO CPU 또는 ONNX Runtime DirectML GPU로 실행하며, 샘플에서 두 결과의 좌표가 같음을 확인했습니다.
- 자세 추론은 1,000프레임마다 checkpoint를 남겨 중단 후 이어서 실행할 수 있습니다.
- 각 세션을 `--session <id>`로 따로 처리할 수 있고, 이미 끝난 작업은 건너뜁니다.

## 주요 산출물

| 파일 | 내용 |
|---|---|
| `points.csv` / `.parquet` | 측정 구간 1개 = 1행: 명령 각도·PWM·압력 12채널 통계·자세 통계 |
| `point_pressure.csv` | 측정 구간 × 센서: 유효 표본 수, 평균·표준편차·범위, 0값·포화 비율 |
| `pose_samples.parquet`, `pose_unique.parquet` | 10 Hz 자세 샘플 / 중복 캡처 제거본 |
| `aligned_10hz.parquet` | 100 ms 구간마다 압력 평균 + 최근접 자세 + 최근 기구 상태 |
| `pose_overlay.mp4` | 중앙 크롭 10 fps 골격 오버레이 영상 |
| `quality_report.json`, `validation.json` | 오류·시계 드리프트·자세 결측 검사, 출력 재검증 |

## 해석할 때 주의할 점

- 압력은 ADC 전압이며 힘(N)이나 압력(kPa)으로 환산하지 않았습니다.
- 기구 각도·PWM은 명령 상태이며, 실제 축 각도나 인체 자세와 같지 않습니다.
- `aligned_10hz`는 오프라인 최근접 결합이라 미래 샘플을 고를 수 있으므로, 실시간 예측기의 입력으로 그대로 쓰지 않습니다.
- 한 번의 수집에서 얻은 관찰값이며, 표준편차는 구간 안 분산이지 반복 실험 간 분산이 아닙니다.

> 모델과 데이터는 저장소에 포함하지 않습니다.
