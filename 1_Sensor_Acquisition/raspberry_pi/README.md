[← 1. 센서 데이터 수집](../README.md) &nbsp;|&nbsp; **① 센싱** → ② 처리 → ③ RL 제어 → ④ 시트 구동

# Raspberry Pi 주행 데이터 수집기 (OASIS DAQ)

> Raspberry Pi 5 한 대로 **GPS + IMU + A필러 카메라 + MediaPipe 자세**를 하나의 시간축으로 외장 SSD에 기록합니다.
> 노트북 대시보드는 언제 연결하거나 끊어도 기록에 영향을 주지 않습니다.

```text
GPS + IMU + A필러 카메라 ──► Raspberry Pi 5 ──► 외장 SSD: OASIS_DAQ/
                                  │
                                  └── (선택) http://<pi>:8080 대시보드
                                       START/STOP · 미리보기 · 골격 · GPS 궤적 · IMU · 수신율 · SSD 상태
```

---

## 설계 핵심

| 원칙 | 구현 |
|---|---|
| **녹화 경로에서 디코딩하지 않는다** | 카메라는 인코딩된 MJPEG 버퍼를 그대로 `ffmpeg -c copy`로 저장합니다 |
| **자세 추정이 녹화를 방해하지 않는다** | MediaPipe는 별도 **프로세스**에서 최신 프레임만 받아 처리합니다. 실측: 자세 프로세스가 코어 60 %를 쓰는 동안 녹화는 101 fps 유지 |
| **센서 하나의 고장이 다른 기록을 멈추지 않는다** | GPS · IMU · 카메라가 각자 독립 스레드에서 동작합니다 |
| **모든 기록에 같은 시계** | 모든 행에 `CLOCK_MONOTONIC_RAW`를 기록하고, 수집 중에는 리샘플링하지 않습니다 |
| **전원이 끊겨도 데이터를 잃지 않는다** | 버퍼 기록 + 주기적 fsync, 끝난 영상 조각은 즉시 fsync, 비정상 종료는 `INCOMPLETE` 표시로 감지 |

### 스레드 구조

```text
capture thread         카메라 읽기 → 최신 프레임 슬롯 → 제한된 큐        (파일 작업 없음)
oasis-camera-writer    ffmpeg 파이프 + frame_timestamps.csv
oasis-segment-sync     끝난 영상 조각 fsync
oasis-ffmpeg-stderr    ffmpeg 오류 출력 비우기 (막히면 캡처가 멈추므로)
oasis-pose (프로세스)   MediaPipe 추론만 담당
gps / imu threads      각자 시리얼 포트와 기록기
oasis-health           health.csv와 임계값 감시
oasis-dashboard        HTTP, 읽기 전용 관찰 + START/STOP
```

## 하드웨어 (실측 기준)

| 구분 | 사양 |
|---|---|
| 호스트 | Raspberry Pi 5, Ubuntu 24.04 (aarch64) |
| GPS | u-blox 7, USB CDC-ACM, NMEA 1 Hz |
| IMU | WITMOTION (WIT 프로토콜, CH340), 9600 baud · 10 Hz |
| 카메라 | Arducam OV9281 USB, MJPG 1280×800 @ 100 fps 유지 |
| 자세 | MediaPipe `pose_landmarker_lite`, 약 13 fps, 지연 p50 66 ms / p95 94 ms |
| 저장 | Samsung T7 외장 SSD (exFAT) |

측정 명령과 결과 전체: [`docs/HARDWARE_BRINGUP.md`](docs/HARDWARE_BRINGUP.md)

## 구성

```text
raspberry_pi/
├── configs/daq.yaml          실제 하드웨어에서 확인한 설정값
├── src/oasis_daq/
│   ├── clock.py              CLOCK_MONOTONIC_RAW (대체 경로 포함)
│   ├── devices.py            VID/PID와 by-id로 장치 식별 (ttyUSB0 같은 번호에 의존하지 않음)
│   ├── storage.py            세션 폴더, 버퍼 기록기, 주기적 fsync
│   ├── session.py            메타데이터, manifest, 요약, GPS 궤적 산출물
│   ├── health.py             /proc · /sys 기반 상태 수집 (psutil 없이)
│   ├── workers.py            독립 작업 스레드, 크기 제한 큐, 최신값 슬롯
│   ├── supervisor.py         상태 머신과 세션 생명주기
│   ├── cli.py                명령줄 도구 oasis-daqctl
│   ├── gps/                  자체 NMEA 파서 (체크섬 엄격 검사) + 수집기
│   ├── imu/                  WIT 프로토콜 + baud 자동 탐색 + 수집기
│   ├── camera/               V4L2 캡처, ffmpeg 분할 녹화
│   ├── pose/                 자세 특징, 중립 자세 보정, 자세 추정 프로세스
│   └── dashboard/            HTTP API + 단일 페이지 UI
├── scripts/                  하드웨어 점검, 자세 모델 받기, 설치/제거, 장시간 시험
├── systemd/                  oasis-daq 서비스
├── udev/                     /dev/oasis_{gps,imu,camera} 고정 이름 규칙
└── tests/                    단위 시험 143개 (하드웨어 불필요)
```

## 실행 방법

```bash
python -m pip install -e .                    # 가상환경에 설치
bash scripts/fetch_pose_model.sh lite         # MediaPipe 모델 받기
sudo bash scripts/install.sh                  # udev 규칙 + systemd 서비스 등록
oasis-daqctl check --probe --probe-imu-baud   # 저장소·장치·시스템 점검

sudo systemctl start oasis-daq
oasis-daqctl start --notes "run1"             # 기록 시작
oasis-daqctl stop                             # 기록 정지
oasis-daqctl summarize <세션 폴더>             # 세션 요약
```

## 검증

```bash
python -m pytest tests/ -q
```

하드웨어 없이 NMEA 체크섬·좌표·단위 변환(표준 벡터와 실제 수신 문장), WIT 프레이밍·체크섬·스케일·재동기화(실제 IMU 바이트),
자세 특징과 중립 보정, 저장 안전성·기록 내구성·타임스탬프 감사를 검사합니다.
일부 시험(udev 심볼릭 링크, `os.getloadavg`)은 Linux 전용입니다.

## 알려진 한계

- IMU는 공장 설정 9600 baud에서 **10 Hz**라 급격한 과도 응답을 잡기에는 느립니다.
- 실시간 중립 자세 보정은 차 안에서 고정되지 않아, 실차 데이터의 중립 기준은 오프라인에서 계산했습니다.
- 수집기의 자세 추정은 CPU(MediaPipe)에서 동작합니다. 최종 제어기의 RTMPose + Hailo 경로는 [`3_RL_Control/deploy`](../../3_RL_Control/deploy/README.md)에 있습니다.
- 대시보드에는 인증이 없으므로 신뢰할 수 있는 네트워크에서만 사용합니다.

## 참고 문서

| 문서 | 내용 |
|---|---|
| [ARCHITECTURE](docs/ARCHITECTURE.md) | 프로세스·스레드, 역압, 시간, 내구성 설계 |
| [HARDWARE_BRINGUP](docs/HARDWARE_BRINGUP.md) | 모든 하드웨어 측정과 그 명령 |
| [DATA_SCHEMA](docs/DATA_SCHEMA.md) | 모든 파일과 열 정의 |
| [OPERATIONS](docs/OPERATIONS.md) | 사용법, 설정, 문제 해결, 복구 |
| [SAFETY](docs/SAFETY.md) | 차량·전기·데이터 손실·개인정보 안전 |
| [TEST_RESULTS](docs/TEST_RESULTS.md) | 실제로 수행한 시험과 수행하지 못한 시험 |
| [BLOCKERS](docs/BLOCKERS.md) | 해결하지 못한 문제와 해결 방법 |

> 기록한 세션 데이터는 저장소에 포함하지 않습니다.
