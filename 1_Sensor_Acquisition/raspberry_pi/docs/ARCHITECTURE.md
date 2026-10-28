[← Raspberry Pi 센서 수집기](../README.md)

# 아키텍처

> 수집기(OASIS DAQ v0.1)의 프로세스·스레드 구조, 역압(back-pressure) 정책, 시간 기준, 내구성 설계를 설명합니다.
> 핵심은 "녹화 경로는 어떤 부하에도 흔들리지 않는다"입니다.

---

## 1. 동작 모드

코드는 하나이고, 노트북 연결 여부와 관계없이 같은 방식으로 동작합니다.

```text
단독 동작                              노트북 연결 시
─────────                              ──────────────
GPS + IMU + A필러 카메라               Raspberry Pi (그대로 녹화 중)
        │                                       │
   Raspberry Pi 5                               └── Ethernet / Wi-Fi
        │                                                │
 공통 단조 시계 타임스탬프                         노트북 웹 브라우저
        │                                         (START/STOP, 미리보기,
 외장 SSD: OASIS_DAQ/                              골격, GPS, IMU,
                                                   수신율, SSD, 지도)
```

노트북은 **버튼 두 개가 달린 관찰자**입니다. 녹화 상태는 전부 Pi 안에 있습니다.
브라우저를 닫거나, Wi-Fi가 끊기거나, HTTP 서버가 죽어도 진행 중인 세션에는 영향이 없습니다.
세션 도중 8080 포트를 차단하는 시험과 MJPEG 스트림을 강제로 끊는 시험으로 확인했습니다([TEST_RESULTS](TEST_RESULTS.md)).

## 2. 프로세스와 스레드 구성

```text
oasis-daq.service  (systemd, 전용 사용자 계정)
└── oasis-daqctl daemon                     ← 모든 장치를 소유하는 단일 프로세스
    ├── Supervisor                          상태 머신, 세션 생명주기
    ├── thread  gps_worker                  시리얼 → raw_nmea.log + gps.csv
    ├── thread  imu_worker                  시리얼 → raw_*.bin + imu.csv
    ├── thread  camera_worker               V4L2 → ffmpeg 파이프 + 타임스탬프
    │   └── subprocess ffmpeg               MJPEG -c copy → 분할 Matroska
    ├── thread  pose_worker (부모 측)       프레임 전달, 결과 기록
    │   └── PROCESS oasis-pose              MediaPipe 추론만 담당
    ├── thread  health monitor              health.csv, 임계값, 이벤트
    └── thread  dashboard HTTP server       읽기 전용 관찰 + START/STOP
```

### MediaPipe를 스레드가 아닌 별도 프로세스로 둔 이유

Pi 5에서 자세 추정은 프레임당 약 66–83 ms의 네이티브 연산을 씁니다. 실제 세션 중 측정값은 다음과 같습니다.

| 프로세스 | CPU 사용 | 결과 |
|---|---|---|
| 자세 추정 자식 프로세스 | 코어 1개의 **약 60 %** | 추론 전담 |
| 메인 프로세스 (카메라 보유) | 코어 1개의 **약 11 %** | 캡처 **101 fps** 안정 유지 |

추론을 수집 프로세스 안에 두면 녹화 FPS가 비전 부하에 좌우됩니다. 이는 설계 요구사항에서 허용하지 않습니다.

자식 프로세스는 가장 최신 JPEG만 받고 단순한 튜플만 돌려줍니다. 파일 쓰기는 모두 부모 프로세스가 하므로,
자식은 입력에 대한 순수 함수이며 죽더라도 원시 데이터 기록에는 영향이 없습니다.

### 나머지 작업자를 스레드로 둔 이유

GPS(초당 2행), IMU(10 Hz), 카메라 기록기는 I/O 위주이며 `serial.read()`, `VideoCapture.read()`, `pipe.write()` 안에서 GIL을 놓습니다.
각자 독립된 `BaseWorker` 스레드에서 자체 `try/except`로 동작합니다. 예외가 난 작업자는 오류를 기록하고
자신을 not-running으로 표시한 뒤 멈출 뿐, **다른 센서의 기록을 멈추지 않습니다.**

## 3. 녹화 경로는 디코딩하지 않는다

```text
V4L2 (MJPEG) ──read()──> 인코딩된 JPEG 바이트 ──┬── EOI까지 패딩 제거 ──> ffmpeg -c copy
   CAP_PROP_CONVERT_RGB = 0                     ├── LatestSlot → 자세 추정 프로세스
                                                └── LatestSlot → 대시보드 미리보기
```

- `CAP_PROP_CONVERT_RGB=0`으로 설정하면 OpenCV가 디코딩하지 않고 카메라의 JPEG 버퍼를 그대로 넘겨줍니다.
  따라서 녹화는 바이트 복사이며, 디코딩·재인코딩·색 변환이 없습니다.
- 두 소비자(자세 추정, 미리보기)는 같은 바이트를 **최신값 우선 슬롯(newest-wins)** 으로 받습니다.
  느린 소비자가 캡처에 역압을 걸 수 없습니다.
- UVC는 버퍼를 고정 크기로 채워 보내므로, 기록 전에 마지막 `FFD9`(EOI) 마커 뒤의 패딩을 잘라냅니다.
  이 처리가 없으면 ffmpeg가 매 프레임 `EOI missing, emulating` 경고를 냅니다.

## 4. 역압(back-pressure) 정책

| 경로 | 정책 | 이유 |
|---|---|---|
| GPS / IMU 행 | 크기 제한이 큰 큐, 넘치면 버린 수를 집계 | 원시 데이터는 잃으면 안 됨. 넘침은 이벤트로 남기고 절대 멈추지 않음 |
| 카메라 → 기록기 | 파이프에 직접 쓰기 | 기록기가 막히는 것은 세션 치명 오류이며 덮어서 숨길 대상이 아님 |
| 카메라 → 자세 추정 | `LatestSlot`, 최신값 우선 | 지난 프레임은 가치가 없으므로 건너뛰는 것이 맞음 |
| 카메라 → 미리보기 | `LatestSlot`, 최신값 우선 | 위와 같음. 녹화에 절대 영향을 주면 안 됨 |
| 자세 → 대시보드 | `LatestSlot` | 표시 전용 |

## 5. 시간 기준

- 모든 기록에 `CLOCK_MONOTONIC_RAW` 기반 `host_monotonic_ns`와 사람이 읽기 위한 `host_wall_time_ns`가 함께 들어갑니다.
  실제 사용한 시계 소스는 `metadata.json`에 기록되며, 대체 경로는 `time.monotonic_ns`입니다.
- 수집 중에는 리샘플링·보간을 하지 않습니다. 각 소스는 자기 주기로 기록되고, 정렬은 후처리에서 합니다.

### 타임스탬프는 도착 시점에 최대한 가깝게

시리얼 포트는 이미 도착한 바이트만 읽습니다(`in_waiting`, 10 ms 폴링).
버퍼가 찰 때까지 블로킹하는 방식(`serial.read(256)` + 1 s 타임아웃)은 1초 분량의 IMU 프레임에 같은 타임스탬프를 찍습니다.

| 읽기 방식 | 중복 타임스탬프 | IMU 지터 p95 |
|---|---|---|
| `serial.read(256)`, 1 s 타임아웃 | 809 샘플 중 669개 | 84.8 ms |
| `in_waiting` + 10 ms 폴링 (현재) | 0개 | 3.2 ms |

`session_summary.json`은 스트림마다 단조성·중복·공백·주기·지터를 감사(audit)합니다.
이런 종류의 회귀가 생기면 분석을 조용히 망치는 대신 요약 파일에 드러납니다.

## 6. 내구성

`flush()`는 커널 페이지 캐시까지만 보냅니다. 장시간 시험 중 USB SSD가 빠졌을 때
**모든 CSV의 데이터 행이 0개, 영상 파일이 0바이트**로 남아 약 10분 분량이 사라진 것이 확인되었습니다. 그래서 다음과 같이 처리합니다.

| 대상 | 처리 |
|---|---|
| CSV / JSONL 기록기 | 주기적 `fsync` (`storage.fsync_interval_s`, 15 s) |
| IMU 원시 바이트 아카이브 | 같은 주기 + 조각(chunk)을 닫을 때마다 `fsync` |
| 끝난 영상 조각 | ffmpeg가 `segments.csv`에 올리는 즉시 `fsync` |
| 진행 중 영상 조각 | 조각 길이 60 s가 최악의 영상 손실 범위 |

- 세션을 열 때 `INCOMPLETE` 파일을 만들고, 정상 종료 시에만 지웁니다. 중단된 세션을 완료된 세션으로 오인할 수 없습니다.
- 새 세션은 항상 새 디렉터리를 받습니다. 비정상 종료된 데몬이 예전 디렉터리에 이어 쓰지 않습니다.

## 7. 상태 머신

```text
INIT ──prepare()──> READY ──start_session()──> RECORDING ──stop_session()──> SAVING ──> READY
  │                   │                            │
  └── 쓸 수 있는 ─────┴──── 장치/디스크/기록기 치명 오류 ──────────────────> ERROR
      저장소 없음
```

- `SAVING`: 작업자 정지 → 경로 산출물 기록 → 타임스탬프 감사 → 요약·manifest 작성 → `INCOMPLETE` 삭제.
- 치명 조건(기록기 종료, 디스크가 정지 임계값 미만, 카메라 분리)이 생기면 세션을 정상적으로 닫은 **뒤에** `ERROR`로 갑니다.
  데이터 마무리가 항상 먼저입니다.

## 8. 장치 식별

커널 장치 이름(`ttyUSB0` 등)은 식별자로 쓰지 않습니다. 다음 순서로 장치를 찾습니다.

1. 설정 파일의 명시적 지정
2. udev 심볼릭 링크 (`/dev/oasis_gps`, `/dev/oasis_imu`, `/dev/oasis_camera`)
3. USB VID/PID + `/dev/serial/by-id`

일치하는 장치가 여럿이면 추측하지 않고 오류로 처리합니다. 확인된 장치 정보는 모든 세션의 `metadata.json`에 기록됩니다.

## 9. v0.1이 의도적으로 하지 않는 것

- 수집 중 보간·리샘플링
- GPS 수신기 설정 변경 (확인된 공장 설정 그대로 기록)
- IMU 설정 쓰기 (baud 탐색은 읽기 전용)
- Hailo 의존 — MediaPipe는 XNNPACK으로 CPU에서 동작
- 맵 매칭·경로 탐색·내비게이션 — 지도는 기록된 궤적만 표시
- `/etc/fstab` 변경, 포맷·파티션·삭제 작업
