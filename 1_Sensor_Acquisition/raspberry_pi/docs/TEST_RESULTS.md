[← Raspberry Pi 센서 수집기](../README.md)

# 시험 결과 (OASIS DAQ v0.1)

> 대상 하드웨어(Raspberry Pi 5)에서 **실제로 수행한** 시험의 결과만 기록합니다. 추정치는 없습니다.
> 수행하지 못한 시험은 [§12](#12-수행하지-못한-시험)에 정리하고 [알려진 제약](LIMITATIONS.md) 문서로 연결합니다.

---

## 시험 환경

| 항목 | 값 |
|---|---|
| 호스트 | Raspberry Pi 5, Ubuntu 24.04.4, 커널 6.8.0-1047-raspi, aarch64 |
| Python | 3.11.16 (가상환경) |
| 저장 | Samsung T7 외장 SSD (exFAT, USB 2.0 포트) |
| 전원 | 5 V / 3 A USB-C PD 어댑터 ([LIMITATIONS 7](LIMITATIONS.md#7-5-v--3-a-전원-용량-부족) 참고) |

## 결과 요약

| 영역 | 결과 |
|---|---|
| 단위 시험 | **143개 통과**, 하드웨어 불필요 |
| 장치 식별 | GPS · IMU · 카메라 자동 식별, udev 고정 이름이 실제 재연결 후에도 유지 |
| 32분 31초 내구 시험 | 모든 작업 스레드 **오류 0**, 스로틀링 없음, 최고 SoC 53.5 °C |
| 최종 검증 세션 | 오류·손실 **0 / 0**, 100 ms 초과 캡처 정지 **0회** |
| SIGKILL 내구성 | 모든 CSV가 강제 종료 1–5초 전까지 보존, 진행 중이던 영상 조각까지 재생 가능 |
| 원시 데이터 재생 | IMU 1534 샘플 × 6필드 재파싱 결과 **불일치 0** |
| 대시보드 분리 | 스트림 강제 종료·포트 차단 중에도 기록 지속 |
| 발견·수정한 결함 | **29건** ([§11](#11-시험-중-발견하고-수정한-결함)) |

---

## 1. 단위 시험 — 143개 통과

```bash
python -m pytest 1_Sensor_Acquisition/raspberry_pi/tests/ -q
# 143 passed
```

| 파일 | 검사 내용 |
|---|---|
| `test_nmea.py` | 체크섬(정상, 본문 손상, 잘못된/누락된 접미사), 남·서반구 포함 위경도 변환과 범위 밖 분 값, 노트→m/s, RMC 필드 순서, GGA→RMC HDOP 전달, GSA DOP, VTG 속도 우선, 실제 미측위 문장, 쓰레기 입력에서 예외 없음 |
| `test_wit.py` | **실제 IMU에서 캡처한 바이트**로 프레이밍 검증, 체크섬 연산, 체크섬·본문 손상, 쓰레기 뒤 재동기화, 1바이트씩 전달, 본문 안의 `0x55`, 스케일 상수, 정지 시 1 g, 원시 카운트 보존, 장착 회전, 자이로 바이어스, 샘플 유효성 |
| `test_features.py` | 중심점, 몸통 각도 부호, 누락·저가시성 랜드마크, 이동량과 기울기, 스케일 불변성, 호스트 시계 기준 기울기 속도, 중립 보정 수락/거부/프레임 부족 |
| `test_storage_session.py` | 세션 폴더 재사용 금지, 대체 경로 선택, CSV/JSONL 기록기, 이중 close, 원시 청크 교체와 교체 전 flush, 타임스탬프 감사(완전한 스트림, 역행, 중복, 공백), GeoJSON 경도·위도 순서, haversine 거리, manifest 해시, 원자적 JSON |
| `test_devices.py` | 커널 이름보다 by-id 우선, 잘못된 VID/PID 거부, **빈 매처는 추측하지 않음**, **모호한 일치는 오류**, 시리얼 번호로 구분, 카메라 이름 매칭, 메타데이터 노드 제외 |
| `test_supervisor_policy.py` | READY/대체 경로 기동, 여유 공간 임계값 미만이면 시작 거부, 대체 경로 거부, **세션 시작마다 저장 경로 재확인**, INCOMPLETE 생명주기, 중복 시작 거부, 세션 없는 정지, 세션 간 폴더 공유 금지, 메타데이터 내용 |

---

## 2. 장치 식별

```text
$ oasis-daqctl check --probe --probe-seconds 4 --probe-imu-baud
== storage ==
  [OK] storage root: <SSD>/OASIS_DAQ (363.1 GB free)
  [OK] free space: 363.1 GB free, 20.0 GB required
== devices ==
  [OK] gps: /dev/serial/by-id/usb-u-blox_AG_..._GPS_GNSS_Receiver-if00 (1546:01a7)
  [OK] imu: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 (1a86:7523)
  [OK] camera: /dev/video0 (Arducam OV9281 USB Camera: Ardu)
== pose model ==
  [OK] pose model: ~/.local/share/oasis-daq/models/pose_landmarker_lite.task
== system ==
  [OK] memory: 6090 MB available of 7933 MB
  [OK] cpu temperature: 41.9 C
  [OK] throttling: vcgencmd get_throttled=0x0
== live probes ==
  [OK] camera stream: 90.3 fps, 362/362 valid JPEG, mean 36894 B
  [OK] gps stream: 42 sentences, 0 checksum errors, fix=no,
       types=['GPGGA','GPGLL','GPGSA','GPGSV','GPRMC','GPTXT','GPVTG']
  [WARN] gps fix: no position fix (normal indoors; needs sky view)
  [OK] imu baud scan: selected 9600 baud

0 problem(s), 1 warning(s)
```

장치 탐색은 `/dev/video1`(UVC 메타데이터 노드)과 `pispbe`/`rpivid` 플랫폼 노드를 올바르게 제외합니다.

### udev 고정 이름

```text
$ sudo bash 1_Sensor_Acquisition/raspberry_pi/scripts/install.sh && ls -l /dev/oasis_*
/dev/oasis_camera -> video0
/dev/oasis_gps    -> ttyACM0
/dev/oasis_imu    -> ttyUSB0
```

**실제 재연결로 검증했습니다.** 카메라가 USB 버스에서 떨어졌다가 새 장치로 다시 잡혔을 때([§9](#9-실제-하드웨어-장애-대응)),
udev가 `/dev/oasis_camera`를 새 노드로 자동 재생성했습니다. `/dev/video0`을 코드에 고정했다면 바로 이 상황에서 실패합니다.

---

## 3. 센서별 검증

### GPS — u-blox 7

| 항목 | 결과 |
|---|---|
| 수신기 식별 | `HW UBX-G70xx`, `PROTVER 14.00`, `ANTSTATUS=OK` |
| 문장 종류 | `GGA/RMC/VTG/GSA/GSV/GLL/TXT`, 1 Hz → 초당 2행 |
| 체크섬 실패 | 전체 실행 합계 **0** |
| 측위 | 실내 벤치에서는 미측위(`GPGGA` 품질 `0`, `GPRMC` 상태 `V`, 위성 3–4개 SNR 22–23 추적) → [LIMITATIONS 1](LIMITATIONS.md#1-gps-측위--실차에서-해결) |

표준 벡터로 파서를 검증했습니다.

| 입력 | 출력 |
|---|---|
| `4807.038,N` | `48.117300` |
| `01131.000,E` | `11.516667` |
| RMC `022.4` 노트 | `11.5235 m/s` |
| VTG `005.5` 노트 | `2.8294 m/s` |
| RMC `230394` + `123519` | `1994-03-23T12:35:19.000Z` |
| 정상 문장의 1글자 손상 | 거부 |

### IMU — WIT 프로토콜, baud 자동 탐색

| baud | 바이트 | 유효 프레임 | 체크섬 오류 | 버린 바이트 | 프레임 비율 | 판정 |
|---|---|---|---|---|---|---|
| **9600** | 1220 | **110** | 0 | 0 | **0.992** | **선택** |
| 115200 | 3072 | 0 | 0 | 3062 | 0.000 | 거부 |
| 57600 | 3072 | 0 | 0 | 3062 | 0.000 | 거부 |
| 38400 | 2304 | 0 | 0 | 2294 | 0.000 | 거부 |
| 19200 | 1670 | 0 | 0 | 1660 | 0.000 | 거부 |

잘못된 baud에서도 바이트는 들어오지만 유효 프레임은 0개입니다. 그래서 탐색은 "바이트 수신 여부"가 아니라 **프레임 유효성**으로 점수를 매깁니다.

실제 프레임 디코딩 결과: 정지 상태 `|a| = 0.9971 g`, 자이로 ≈ 0, 온도 25.95 °C.
패킷 구성 `acc/gyro/angle/mag` 각각 **10 Hz**, 모든 실행에서 **체크섬 실패 0** (10.8분 세션 한 번에 6504 샘플).

### 카메라 — Arducam OV9281

| 모드 | 결과 |
|---|---|
| `YUYV` | 10 fps 상한 — 사용 불가 |
| `MJPG 1280×800` | ffmpeg 패스스루로 10초간 1002프레임 = **100.2 fps** 유지, DAQ 내부에서 약 101 fps |
| 30 fps 요청 | 100 fps로 동작 (장치가 100/120 fps만 제공) |

통합 전 녹화 경로 단독 검증: 1169프레임 캡처 → 4프레임마다 1프레임씩 292프레임 기록 → 조각 3개에 정확히 125+125+42 = 292프레임, 모두 재생 가능.

### MediaPipe — 흑백 입력 경로

카메라가 흑백이므로 자세 입력은 흑백→3채널 변환을 거칩니다. MediaPipe 공식 샘플 사진으로 두 경로를 비교했습니다.

| 경로 | 랜드마크 | 왼쪽 어깨 | 어깨 너비 | 몸통 각도 |
|---|---|---|---|---|
| 컬러 (기준) | 33 | (0.5408, 0.4820) v=1.000 | 0.0860 | +1.48° |
| **흑백→RGB (OV9281 경로)** | **33** | (0.5378, 0.4811) v=1.000 | 0.0799 | +1.19° |

정규화 좌표 차이 <0.01, 가시성 >0.96으로 흑백 경로를 그대로 사용할 수 있습니다.

Pi 5 단독 처리량 (`pose_landmarker_lite`):

| 입력 크기 | 유지 fps | p50 | p95 |
|---|---|---|---|
| 640×400 | 13.3 fps | 66 ms | 94 ms |
| 256×256 | 12.4 fps | 69 ms | 101 ms |

벤치 카메라 앞에는 사람이 없어 벤치 세션에서는 `frames_with_pose = 0`입니다. 실차 결과와 중립 보정 한계는 [LIMITATIONS 2](LIMITATIONS.md#2-실시간-중립-자세-보정)에 있습니다.

---

## 4. 통합 세션

센서 3종 + 자세 추정을 systemd 데몬으로 실행하며 SSD에 기록했습니다.

| | 83초 세션 | 10.8분 세션 (실제 장치 이탈 포함) |
|---|---|---|
| 카메라 | 8027프레임 @ 98.9 Hz, 2006프레임 기록, 무효 0 | 64612프레임 @ 99.9 Hz, 무효 0 |
| IMU | 809 샘플 @ 9.96 Hz, 체크섬 오류 0 | 6504 샘플 @ 10.05 Hz, 오류 0 |
| GPS | 164행 @ 2.02 Hz, 체크섬 오류 0 | 1292행 @ 2.00 Hz, 오류 0 |
| 자세 | 768프레임 @ 9.46 Hz, p95 110 ms | 6343프레임 @ 9.82 Hz |
| 오류 | 0 | 1 (카메라 이탈) |

### 타임스탬프 감사

모든 세션에서 모든 스트림이 `monotonic = true`, 역행 없음 (10.8분 세션):

```text
gps     n= 1292   1.998 Hz  dup=0  gaps=0
imu     n= 6504  10.047 Hz  dup=4  gaps=1
camera  n=64612  99.919 Hz  dup=0  gaps=11
pose    n= 6343   9.817 Hz  dup=0  gaps=10
```

GPS는 GGA와 RMC가 1초에 한 번 약 5 ms 간격으로 몰려 들어오는 수신기 특성 때문에, 전체 행 기준으로는 공백과 약 1000 ms의 p95 지터가 보입니다. 이 문제는 감사 방식을 고쳐 해결했습니다([§8](#8-독립-코드-리뷰)).

### 직렬 읽기 묶음 타임스탬프 (결함 #3)

`serial.read(256)`에 1초 타임아웃을 주면 버퍼가 찰 때까지 블로킹되어, 1초 분량의 프레임이 같은 타임스탬프를 받습니다.
작업 스레드는 이제 도착한 만큼만 읽습니다(`in_waiting`, 10 ms 폴링).

| | 수정 전 | 수정 후 |
|---|---|---|
| IMU 중복 타임스탬프 | 669 / 809 | **0** |
| IMU 지터 p95 | 84.77 ms | **3.22 ms** |

이 문제는 모든 `session_summary.json`에 포함되는 타임스탬프 감사로 검출되었습니다.

---

## 5. 내구성

### `fsync` 도입 배경

외장 SSD가 기록 중 USB에서 분리되며 시스템이 재부팅된 사례([§9](#9-실제-하드웨어-장애-대응))에서, 세션은 올바르게 `INCOMPLETE`로 표시되었지만
기록 22초 시점에 `frames_recorded=505`가 확인되었음에도 모든 CSV가 데이터 0행, `driver_000.mkv`가 0바이트였습니다.
`flush()`는 커널 페이지 캐시까지만 도달하며, `dmesg`에는 `Synchronize Cache(10) failed`와 읽기 전용 재마운트가 남았습니다.

| 대책 | 설정 |
|---|---|
| 모든 기록기 주기적 `fsync` | `storage.fsync_interval_s: 15.0` |
| 완료된 영상 조각 즉시 `fsync` | 전용 스레드 `oasis-segment-sync` |
| 영상 조각 길이 120 s → 60 s | `camera.segment_seconds: 60` |
| 세션 시작마다 저장 경로 재확인 | `_refresh_storage_root` |

### SIGKILL 검증 (154초 세션, 예고 없이 강제 종료)

| 파일 | 복구 | 마지막 행 |
|---|---|---|
| `gps/gps.csv` | 297행 | 종료 5초 전 |
| `imu/imu.csv` | 1535행 | 종료 1초 전 |
| `camera/frame_timestamps.csv` | 14970행 | 종료 4초 전 |
| `pose/features.csv` | 1474행 | 종료 2초 전 |
| `health/health.csv` | 149행 | 종료 5초 전 |
| `imu/raw_000.bin` | 67497 바이트 | — |
| `gps/raw_nmea.log` | 792줄 | — |

Matroska 조각 3개 모두 재생 가능하며 **진행 중이던 조각도 포함**됩니다(`ffprobe` 기준 1500 + 1500 + 821프레임).
`INCOMPLETE` 표시가 남았고, systemd가 데몬을 자동 재시작했습니다(`NRestarts=1`, 서비스 `active`).

실제 전원 차단 시험은 수행하지 않았습니다 → [LIMITATIONS 5](LIMITATIONS.md#5-전원-차단-내구성--부분-검증).

### 캡처 스레드 정지 제거 (결함 #6, #21)

캡처 스레드가 파일 작업을 직접 하면 60초 조각 경계마다 캡처가 멈춥니다. exFAT/USB 2.0에서 56 MB 조각을 fsync하는 데 그만큼 걸리기 때문입니다.

```text
t_since_start   gap_ms    가장 가까운 조각 경계
        60.63    583.3    1.15 s
       120.75    653.6    1.27 s
       ...       ...      ...
       601.21    700.4    0.90 s
```

fsync를 별도 스레드로 옮긴 뒤에도 ffmpeg로의 블로킹 `stdin.write()`와 타임스탬프 CSV fsync가 캡처 스레드에 남아 있었습니다.
현재는 기록 스레드(`oasis-camera-writer`)가 둘 다 맡고, 캡처 스레드는 크기 제한 큐(`camera_write_maxsize: 240`)에 넣기만 합니다.

각 구조에서 조각 교체 3회 이상 측정:

| 캡처 스레드가 하는 일 | 105 ms 초과 정지 | 최대 간격 | 프레임 손실 |
|---|---|---|---|
| fsync 직접 수행 | 60초 조각마다 1회 | 742.7 ms | 1.15 % |
| fsync만 별도 스레드 | 약 217초마다 1회 | 296.9 ms | 0.20 % |
| **큐 삽입만 (현재)** | **0** | **22.9 ms** | **0.039 %** |

현재 구조에서 프레임 간격의 99.961 %가 15 ms 미만이고, 기록 큐 깊이는 0을 넘지 않았으며 `camera_write_drops`는 0입니다.

---

## 6. 원시 데이터 재생

파싱된 CSV와 함께 원시 바이트를 저장하는 이유는, 파서를 고친 뒤 원본 스트림에 다시 적용할 수 있게 하기 위함입니다. SIGKILL 세션으로 이를 검증했습니다.

**IMU** — 67 497바이트 원시 파일을 처음부터 재파싱:

```text
frames: ok=6136  bad_checksum=0  bytes_discarded=0     (100.000% valid)
samples reconstructed from raw : 1534
samples in imu.csv (written live): 1535
compared 1534 samples x 6 raw fields -> 0 mismatches
```

**GPS** — `raw_nmea.log`를 문장 조립기로 재생:

```text
792 sentences, 0 checksum failures
rows replayed from raw : 299
rows in gps.csv        : 297
```

원시 기록기는 파싱보다 먼저 실행되므로 원시 파일은 CSV와 **같거나 더 완전합니다**.
1–2행 차이는 예고 없는 강제 종료에서 예상되는 경계 효과이며, 기록된 데이터는 모두 원시 스트림에서 복원할 수 있습니다.

---

## 7. 장시간 세션

### 7.1 내구 시험 — 32분 31초, 오류 0

systemd 데몬으로 SSD에 기록하면서 대시보드를 계속 폴링했습니다. (기록 스레드 분리 이전 코드)

| 항목 | 결과 |
|---|---|
| 기간 / 최종 상태 | **32:31**, `COMPLETE`, `INCOMPLETE` 제거 |
| **오류** | 모든 작업 스레드 **0** |
| 카메라 | 196 556프레임 @ 100.77 Hz, **49 139프레임 기록 = 정확히 196 556 / 4** |
| IMU | 19 592 샘플 @ 10.04 Hz, **78 368프레임 중 체크섬 오류 0** |
| GPS | 3 894행 @ 2.00 Hz, 약 15 000문장 중 체크섬 실패 4 (0.03 %) |
| 자세 | 19 262프레임 @ 9.88 Hz, 지연 p50 45.1 ms / p95 59.6 ms |
| 저장 | 조각 33개, 1 600 MB → **2.95 GB/h** |
| 타임스탬프 | 모든 스트림 `monotonic = true` |

1초 간격 상태 샘플 1 940개:

| 지표 | 최소 | 평균 | 최대 |
|---|---|---|---|
| CPU | 15.0 % | 27.3 % | 75.2 % |
| SoC 온도 | 44.1 °C | 48.6 °C | **53.5 °C** |
| 메모리 사용 | 21.1 % | 23.9 % | 25.2 % |
| 카메라 캡처 | 82.2 fps | 100.9 fps | 101.5 fps |
| 자세 | — | 9.84 fps | 9.88 fps |
| IMU | — | 10.05 Hz | 10.05 Hz |
| SSD 쓰기 지연 | 4.7 ms | 18.9 ms | 159.9 ms |

시작 전후 `vcgencmd get_throttled = 0x0` — 약 100 fps 캡처와 연속 자세 추정을 32분간 수행해도 저전압·열 스로틀링이 없습니다.

이 세션에서 확인된 사항:

- **캡처 정지**: 1 949초 동안 114–297 ms 정지 9회(약 120초 주기), 누락 403 / 196 556프레임 = **0.20 %**, 간격의 99.883 %가 15 ms 미만. 이후 기록 스레드 분리로 해소([§5](#캡처-스레드-정지-제거-결함-6-21)).
- **가짜 손실 카운터 (결함 #27)**: 요약에 `imu: dropped=11400`이 표시되었지만 `imu.csv`에는 **19 592 샘플이 모두 19 592행으로** 남아 있었습니다. `GpsWorker`·`ImuWorker`가 소비자가 없는 `DropCountingQueue`에 행을 넣어, 큐가 상한 8 192에서 가득 찬 뒤 `dropped`만 증가한 것입니다. 해당 큐를 제거했고, `health.csv`는 실제로 존재하는 카메라 기록 큐만 보고합니다.

### 7.2 최종 검증 세션 — 모든 수정 사항을 하드웨어에서 확인

모든 수정을 반영한 코드로 7분 24초 기록하고, **기록 중** `systemctl restart`로 종료해 정상 SIGTERM 경로도 함께 검증했습니다.

| 항목 | 결과 |
|---|---|
| 최종 상태 | `COMPLETE`, `clean_close: true`, `workers_not_joined: []` |
| **오류 / 손실** | 모든 작업 스레드 **0 / 0** |
| 카메라 | 44 659프레임 @ 100.59 Hz, 11 164프레임 기록(정확히 1/4), `write_queue_drops: 0` |
| **105 ms 초과 캡처 정지** | **0회** — 최대 간격 **22.9 ms**, 99.973 %가 15 ms 미만 |
| IMU | 4 454 샘플 @ 10.03 Hz, **17 816프레임 중 체크섬 오류 0** |
| GPS | 885행 @ 1.99 Hz, **체크섬 실패 0** |
| 자세 | 4 368 @ 9.84 Hz, 지연 p50 41.8 ms / p95 52.9 ms |
| `events.jsonl` | **엄격 JSON 거부 0** (수정 전 세션마다 1건) |
| 경고 | 1건 — `neutral_calibration_failed` (착석자 없음, 정상 동작) |

수정된 타임스탬프 감사:

```text
gps           443 rows   1.000 Hz  mono=True dup=0 gaps=0  jitter p50 0.674 ms p95 16.5 ms
gps_all_rows  885 rows   2.000 Hz  bimodal=true, gap statistics suppressed
imu          4454 rows  10.050 Hz  mono=True dup=1 gaps=1
camera      44659 rows 100.918 Hz  mono=True dup=0 gaps=25
pose         4368 rows   9.885 Hz  mono=True dup=0 gaps=1
```

### 기록 중 정상 재시작 (SIGTERM)

```text
+0.0 s  Stopping oasis-daq.service...
+0.0 s  state: RECORDING -> SAVING (daemon shutdown)
+1.0 s  state: SAVING -> READY (session saved)
+1.0 s  daemon exited cleanly
+2.0 s  Started oasis-daq.service
+7.0 s  state: INIT -> READY
```

SIGTERM에서 세션 마무리까지 1.4초. `INCOMPLETE` 제거, 산출물 6종 모두 기록, 새 데몬은 7초 뒤 READY로 복귀합니다. (로그 시각은 1초 해상도)

이 세션에서 발견한 결함:

- **`clean_close`가 파일에 남지 않음 (결함 #28)**: 요약을 `write_json`으로 직렬화한 뒤에 플래그를 설정해 `session_summary.json`에 빠져 있었습니다. 이제 기록 전에 결정하며, 시험이 파일에 필드가 있는지 확인합니다.
- **이봉(bimodal) 분포의 잘못된 공백 통계 (결함 #29)**: GGA와 RMC가 0.3 ms 간격으로 오고 1초를 쉬는 탓에, 정상인 105초 기록에서 `gps_all_rows`가 `gaps: 103, estimated_missed_samples: 5745`로 보고되었습니다. 이제 간격의 p90/p10 비가 10을 넘는 계열은 `bimodal`로 표시하고 공백 통계를 생략합니다.

---

## 8. 독립 코드 리뷰

별도 검토자가 6개 위험 영역을 기준으로 패키지 전체를 읽고, 파서와 타임스탬프 주장을 docstring이 아닌 실제 세션 CSV와 대조했습니다. 6개 중 5개 영역에서 결함이 발견되었고 모두 수정했습니다.

**결함 없음으로 확인된 사항**

- 모든 IMU 스케일·단위 변환, 모든 NMEA 필드 인덱스와 좌표·속도 변환, `v_veh = R · v_sens` 규약
- 읽기 묶음 수정 후 IMU 호스트 타임스탬프 — 실측 7961행, **역행 0, 중복 5 (0.06 %), Δt 중앙값 99.61 ms**
- 저장 안전성: 포맷·파티션·마운트·언마운트·truncate·기록 데이터 삭제가 없습니다. `unlink`는 탐침 파일 2개와 `INCOMPLETE` 표시에만 사용합니다. `SessionPaths.allocate`는 시계가 거꾸로 가도 폴더를 재사용하지 않습니다.
- 조각 관리: ffmpeg PTS 분할이 60초 조각당 정확히 1500프레임으로 `_frames_per_segment`와 일치
- 자세·미리보기와 캡처의 분리: `LatestSlot`은 대입 4번 동안만 잠금, MediaPipe는 spawn 프로세스, 미리보기는 디코딩 전에 잠금 해제, 녹화 경로는 디코딩하지 않음
- **대시보드 분리**: 대시보드에서 수집 경로로 들어가는 경로가 없습니다. `stop_session` 도중 클라이언트가 끊겨도 정지는 중단되지 않고 마지막 응답 쓰기만 실패합니다.

**심각도 높음으로 분류된 결함 4건** (전체 목록은 [§11](#11-시험-중-발견하고-수정한-결함) #14–26)

| 결함 | 영향 |
|---|---|
| ffmpeg stderr 파이프를 비우지 않음 | 오류 출력 64 KiB가 쌓이면 ffmpeg가 멈추고, 캡처 스레드의 `stdin.write()`가 **영구히** 블로킹 — 센서 지연 경고 외에는 증상 없이 녹화 중단 |
| `SAVING` 중 `shutdown()`이 무동작 | 정지 진행 중 `systemctl stop`이 오면 저장 도중 데몬 스레드가 종료 — 요약·manifest 없음, 기록기 미flush, **종료 코드 0이라 systemd가 재시작하지 않음** |
| `INCOMPLETE` 표시를 무조건 제거 | 작업 스레드 join 실패로 기록기·마지막 조각이 마무리되지 않아도 정상 세션과 구분 불가 |
| udev 경로가 모호성 검사를 우회 | CH340 어댑터가 두 개일 때 잘못된 포트를 IMU로 조용히 사용 — **udev 규칙을 설치한 경우에만** 발생 |

GPS 타임스탬프 감사 수정 전후 (결함 #20):

| | 수정 전 (전체 행) | 수정 후 (GGA 측위 주기 행) |
|---|---|---|
| 측정 주기 | 2.0 Hz | **1.0 Hz** (`rate_error 0.0 %`) |
| 지터 p50 | 964 ms | **0.69 ms** |
| 지터 p95 | 982 ms | **16.0 ms** |
| 공백 임계값 | 2.78 s — 3회 연속 측위 누락을 놓침 | 1.75 s, 3초 누락을 `estimated_missed_samples: 3`으로 보고 |

수신기 자체는 원래 정상이었고, 감사가 잘못된 계열을 측정하고 있었습니다.

---

## 9. 실제 하드웨어 장애 대응

시험 중 계획에 없던 USB 이탈이 두 번 발생했습니다. 둘 다 **5 V / 3 A 전원**과 일치합니다. 이 전원에서는 USB 4개 장치(GPS, IMU, 카메라, SSD)의
총 전류가 600 mA로 제한됩니다. `vcgencmd get_throttled`는 내내 `0x0`이었지만, 이 값은 Pi 입력 레일을 보고할 뿐 USB 포트 전류 예산은 반영하지 않습니다.
→ [LIMITATIONS 7](LIMITATIONS.md#7-5-v--3-a-전원-용량-부족)

### SSD 분리 + 재부팅

`usb 2-2: USB disconnect` → `Synchronize Cache(10) failed` → `exFAT-fs (sda1): Volume was not properly unmounted` 후 재부팅.
SSD는 다른 버스로 다시 잡혔고, **SSD의 기존 데이터는 손상되지 않았습니다.** 이 사례로 두 가지 결함이 드러났습니다.

- `fsync` 부재 (결함 #4) → [§5](#5-내구성)
- 데몬은 부팅 시 시작되지만 SSD는 로그인 후 마운트되어, 데몬이 로컬 대체 경로를 선택한 뒤 다시 확인하지 않음 (결함 #5) → 세션 시작마다 저장 경로 재확인으로 수정 ([LIMITATIONS 4](LIMITATIONS.md#4-외장-ssd-마운트-시점))

### 기록 중 카메라 분리

`usb 2-1: USB disconnect`, `uvcvideo: Failed to resubmit video URB (-19)` 후 카메라가 1초 뒤 새 USB 장치로 다시 잡혔습니다.
**의도하지 않은 센서 분리 시험이 되었고, 시스템은 설계대로 동작했습니다.**

| 요구 사항 | 관찰 결과 |
|---|---|
| 장치 이탈 감지 | 연속 50회 읽기 실패 → `device_lost` 이벤트 |
| 세션 정상 종료 | `RECORDING → SAVING → ERROR`, 사유 기록 |
| 산출물 기록 | summary, manifest, metadata, route, events 모두 존재 |
| `INCOMPLETE` 유지 | 존재 (오류 종료) |
| **다른 센서에 영향 없음** | 카메라 `errors=1`, **GPS · IMU · 자세 모두 `errors=0`** |
| 데이터 보존 | 카메라 64612 / IMU 6504 / GPS 1292 / 자세 6343 샘플 |
| 영상 무결성 | **조각 11개 모두 재생 가능, 손상 0** |
| 재연결 후 고정 이름 | udev가 `/dev/oasis_camera` 재생성 |

---

## 10. 대시보드 · systemd · 자원 사용

### 대시보드

기록 중 같은 LAN의 다른 호스트에서 접속해 시험했습니다.

| 엔드포인트 | 결과 |
|---|---|
| `GET /` | 200, 16 363 B, 5.8 ms |
| `GET /api/status` | 200, 1.7 ms, 작업 스레드 4개의 실시간 값 |
| `GET /api/health` | 200 — CPU 48.4 %, 46.3 °C, 디스크 쓰기 지연 7.15 ms |
| `GET /api/preview.jpg` | 200, 8 048 B, 640×400으로 디코딩 |
| `GET /api/stream.mjpg` | 3초에 23프레임 ≈ 7.7 fps (설정 8 fps) |
| `GET /api/latest`, `/api/route.geojson` | 200 |
| `GET /api/nonexistent` | 404 |

연결이 끊겨도 기록은 멈추지 않습니다.

| 이벤트 | 상태 | 경과 | 기록 프레임 |
|---|---|---|---|
| MJPEG 스트림 전송 중 강제 종료 | RECORDING | 00:00:42 | 1033 |
| iptables로 8080 포트 20초 차단 | RECORDING | 00:01:02 | 1532 |

20초 동안 499프레임 = 기록 속도 25 fps와 정확히 일치, 오류 0. 대시보드가 Pi에 접속하지 못하면 "기록에는 영향 없음" 배너를 표시합니다.

### systemd

| 항목 | 결과 |
|---|---|
| 부팅 시 자동 시작 | 재부팅 후 journal에 `-- Boot ... --` 직후 `Started oasis-daq.service`, `state: INIT -> READY` (조작 없음) |
| `Restart=on-failure` | SIGKILL 후 `active` 복귀, `NRestarts=1` |
| 정상 SIGTERM | 기록 중 `systemctl restart` → `RECORDING -> SAVING -> READY`, `INCOMPLETE` 제거, 산출물 모두 기록 |
| 유닛 파일 | `StartLimitIntervalSec`를 `[Unit]` 섹션에 배치 (`[Service]`에 두면 경고) |
| 오래된 세션 재개 금지 | 시작할 때마다 새 폴더 할당 |

### 자원 사용 (10분 세션, `health.csv` 1 Hz × 611 샘플)

| 지표 | 최소 | 평균 | 최대 |
|---|---|---|---|
| CPU | 13.8 % | 27.6 % | 61.4 % |
| SoC 온도 | 42.5 °C | 46.0 °C | 49.1 °C |
| 메모리 사용 | 20.0 % | 22.3 % | 23.4 % |
| 카메라 캡처 | — | 100.13 fps | 101.28 fps |
| 자세 | — | 9.67 fps | 9.80 fps |
| IMU | — | 10.01 Hz | 10.05 Hz |
| 자세 지연 p50 | 50.2 ms | 50.7 ms | 53.8 ms |

`vcgencmd get_throttled = 0x0` 유지 — 저전압·열 스로틀링 없음. 25 fps 기록 시 저장 사용량 **약 3.25 GB/h**.

**프로세스 분리 실측**: MediaPipe 자식 프로세스가 코어 하나의 약 60 %를 쓰는 동안 메인 수집 프로세스는 약 11 %를 쓰며 101 fps를 유지했습니다. 비전 부하가 녹화 경로에 영향을 주지 않습니다.

---

## 11. 시험 중 발견하고 수정한 결함

모두 시스템을 실제로 실행하거나 그 출력을 읽어서 발견했습니다. 번호는 발견 순서입니다.

| # | 결함 | 발견 경로 | 심각도 |
|---|---|---|---|
| 1 | `BaseWorker._stop`이 `threading.Thread._stop`을 가려 `join()`에서 `TypeError: 'Event' object is not callable` | 첫 통합 실행이 종료 시 비정상 종료 | 치명 — 세션을 정상 종료할 수 없음 |
| 2 | 원시 NMEA·IMU 로그가 close 전까지 flush되지 않음 | 비정상 종료 세션에서 둘 다 0바이트 | 높음 |
| 3 | 직렬 읽기가 1초 분량 프레임에 같은 타임스탬프 부여 | 감사에서 IMU 809 샘플 중 중복 669 | 높음 — 정밀 정렬 불가 |
| 4 | 이동식 exFAT에서 `fsync` 없는 `flush()`는 내구성 없음 | 실제 SSD 분리로 약 10분 수집분 손실 | 치명 |
| 5 | 저장 경로를 부팅 시 한 번만 결정 | 재부팅 후 SSD가 마운트되었는데도 microSD 대체 경로 사용 | 높음 |
| 6 | 조각 `fsync`를 캡처 스레드에서 실행 | 60초 조각마다 583–743 ms 정지 10회, 프레임 1.15 % 손실 | 높음 |
| 7 | 값이 `0`일 수 있는 타임스탬프의 참/거짓 검사로 `lean_velocity` 비활성화 | 단위 시험 | 중간 |
| 8 | 첫 샘플 전에 `sensor_stale` 발생 | 모든 세션에 가짜 GPS 경고 | 낮음 |
| 9 | `events.jsonl`에 `Infinity` 그대로 기록 | 실제 파일을 엄격 JSON 파서가 거부 | 중간 — 후처리 도구 오류 |
| 10 | `git status --porcelain`을 `.strip()`으로 파싱해 경로가 한 글자 밀림 | `metadata.json`에 `cquisition/...` 기록 | 중간 — 출처 정보가 조용히 틀림 |
| 11 | 궤적이 비면 `route_summary()`가 `distance_km`/`start`/`end` 누락 | `summarize`가 `None km` 출력 | 낮음 |
| 12 | udev 심볼릭 링크를 설정된 매처와 대조하지 않고 사용 | 성능 저하 모드 시험에서 엉뚱한 장치 선택 | 높음 — 장치 오인 |
| 13 | RMC `course_deg`를 필드 7이 아닌 필드 8(날짜)에서 읽음 | 파서 시험 작성 중 | 높음 |
| 14 | ffmpeg stderr 파이프 미배출 → 캡처 스레드 교착 | 독립 리뷰 | **높음** |
| 15 | 정지 진행 중 `shutdown()` 무동작 | 독립 리뷰 | **높음** |
| 16 | 스레드 join 실패·산출물 실패 시에도 `INCOMPLETE` 제거 | 독립 리뷰 | **높음** |
| 17 | udev 경로가 모호성 검사 우회 | 독립 리뷰 | **높음** |
| 18 | `device:`만 있는 매처가 모든 포트와 일치 | 독립 리뷰 | 중간 |
| 19 | RMC 행이 이전 측위 주기의 고도·`fix_quality`를 그대로 가져감 | 독립 리뷰 | 중간 |
| 20 | GPS 타임스탬프 감사가 잘못된 계열 측정, 공백 임계값 2.8 s | 독립 리뷰 | 중간 |
| 21 | 블로킹 파이프 쓰기와 CSV fsync가 아직 캡처 스레드에 있음 | 독립 리뷰 | 중간 |
| 22 | 장착 회전 행렬을 검증 없이 수용 (`det = −1`이면 조용히 좌우 반전) | 독립 리뷰 | 중간 |
| 23 | IMU 파싱 예외 처리가 프레임 루프 밖 → 프레임 하나의 오류로 청크 전체 폐기 | 독립 리뷰 | 중간 |
| 24 | 청크 교체 시 보호 없는 `fsync`가 IMU 작업 스레드를 종료시킬 수 있음 | 독립 리뷰 | 중간 |
| 25 | 주석은 마운트 지점 검사라고 했지만 `is_dir()` 사용 | 독립 리뷰 | 낮음 |
| 26 | `lean_velocity`를 추론 결과 시각 기준으로 미분 | 독립 리뷰 | 낮음 |
| 27 | GPS/IMU 행을 소비자 없는 큐에 넣어 가짜 "손실" 11 400건 | 32분 내구 시험 | 중간 — 거짓 손실 신호 |
| 28 | 요약 직렬화 후 `clean_close` 설정 → 파일에 누락 | 최종 검증 세션 | 중간 — 수정 근거가 남지 않음 |
| 29 | 이봉 분포 계열이 정상 데이터에서 공백 103·누락 5745 보고 | 최종 검증 세션 | 중간 — 거짓 손실 신호 |

결함 1–6, 12–13은 실제 주행에서 각각 데이터를 조용히 틀리게 하거나 잃게 만들었을 결함입니다.

---

## 12. 수행하지 못한 시험

| 시험 | 이유 | 참고 |
|---|---|---|
| 실제 전원 차단 시 내구성 | 전원을 물리적으로 제거해야 함 | [LIMITATIONS 5](LIMITATIONS.md#5-전원-차단-내구성--부분-검증) |
| 실시간 중립 자세 보정 고정 | 벤치에 착석자 없음, 실차에서는 고정되지 않음 | [LIMITATIONS 2](LIMITATIONS.md#2-실시간-중립-자세-보정) |
| 실제 디스크 가득 참 시 임계값 정지 | 여유 362 GB, 단위 시험으로 대체 | — |
| GPS · IMU 의도적 분리 | 카메라 분리는 실제로 발생([§9](#9-실제-하드웨어-장애-대응)) | [LIMITATIONS 8](LIMITATIONS.md#8-기타-미검증-항목) |
| GPIO START/STOP 버튼 | 핀 미확정, 설정에서 비활성 | [LIMITATIONS 8](LIMITATIONS.md#8-기타-미검증-항목) |
| Hailo 가속 | 수집기는 CPU MediaPipe 사용, Hailo 의존성 없음 | [LIMITATIONS 8](LIMITATIONS.md#8-기타-미검증-항목) |
