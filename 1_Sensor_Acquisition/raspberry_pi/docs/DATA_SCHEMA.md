[← Raspberry Pi 센서 수집기](../README.md)

# 데이터 스키마

> 수집기가 한 세션에 남기는 모든 파일과 열(column)의 정의입니다.
> 스키마 버전: `schema_version: oasis-daq/0.1`

---

## 1. 세션 폴더 구조

```text
<storage_root>/sessions/YYYY-MM-DD/session_YYYYMMDD_HHMMSS[_<suffix>]/
├── camera/
│   ├── driver_000.mkv          60 s 단위 Matroska 조각, MJPEG 스트림 복사
│   ├── driver_001.mkv
│   ├── segments.csv            ffmpeg가 기록: name,start_s,end_s
│   └── frame_timestamps.csv    캡처한 프레임마다 1행
├── pose/
│   ├── landmarks.jsonl         랜드마크가 검출된 프레임마다 객체 1개
│   └── features.csv            처리한 프레임마다 1행
├── imu/
│   ├── raw_000.bin             가공하지 않은 시리얼 바이트, 약 1 MB 단위 조각
│   └── imu.csv                 조립된 샘플마다 1행
├── gps/
│   ├── raw_nmea.log            줄마다 "<monotonic_ns> <wall_ns> <sentence>"
│   └── gps.csv                 GGA/RMC 문장마다 1행
├── health/
│   └── health.csv              1 Hz 시스템 + 작업자 상태
├── events.jsonl                구조화된 이벤트 (상태 변화, 오류)
├── metadata.json               세션 해석에 필요한 모든 정보
├── manifest.json               파일 목록과 SHA-256
├── session_summary.json        길이, 작업자 통계, 타임스탬프 감사
├── route.geojson               주행 궤적 LineString
├── route_map.html              단독 실행 지도 (OSM + 오프라인 대체 화면)
├── route_summary.json          거리, 경계 상자, 시작·끝 지점
└── INCOMPLETE                  세션이 열려 있거나 비정상 종료된 경우에만 존재
```

`INCOMPLETE`는 복구 표시입니다. 세션을 열 때 만들고 정상 종료 시에만 지웁니다.
**이 파일이 있으면 그 세션은 중단된 것입니다.**

## 2. 공통 열

모든 스트림의 모든 행에 다음 두 열이 들어갑니다.

| 열 | 형식 | 의미 |
|---|---|---|
| `host_monotonic_ns` | int | `CLOCK_MONOTONIC_RAW` ns — **정렬 기준 키** |
| `host_wall_time_ns` | int | UNIX epoch ns, 사람이 읽기 위한 참고용 |

- 실제 시계 소스는 `metadata.json → clock.source`에 기록됩니다.
- 벽시계 시간은 NTP 보정으로 건너뛸 수 있지만 단조 시계는 그렇지 않습니다. **정렬은 항상 `host_monotonic_ns`로 합니다.**
- 수집 중 리샘플링은 없습니다. 각 스트림은 자기 주기로 기록되며, 정렬은 후처리에서 기준 타임라인(보통 카메라 프레임)에 맞춰 합니다.

## 3. `gps/gps.csv`

| 열 | 형식 | 설명 |
|---|---|---|
| `host_monotonic_ns`, `host_wall_time_ns` | int | |
| `gps_utc` | str\|null | RMC 날짜+시각의 ISO-8601 표현, 예: `2026-01-01T00:00:00.000Z` |
| `latitude_deg`, `longitude_deg` | float\|null | 부호 있는 십진 도, WGS-84 |
| `altitude_m` | float\|null | GGA 안테나 고도 (지오이드 기준) |
| `ground_speed_mps` | float\|null | RMC 또는 VTG의 knots → m/s (×0.514444) |
| `course_deg` | float\|null | 진방위. **정지·저속에서는 의미 없음** |
| `fix_quality` | int\|null | GGA 6번 필드: 0 = fix 없음, 1 = GPS, 2 = DGPS, … |
| `fix_valid` | 0/1 | GGA quality ≥ 1 또는 RMC status `A` |
| `satellite_count` | int\|null | GGA의 *사용 중인* 위성 수 (보이는 위성 수 아님) |
| `hdop`, `vdop`, `pdop` | float\|null | HDOP는 GGA/GSA, VDOP/PDOP는 GSA에서만 |
| `source_sentence` | str | 이 행을 만든 문장: `GPGGA` 또는 `GPRMC` |
| `sequence_id` | int | 세션 내 단조 증가 카운터 |
| `checksum_valid` | 0/1 | 항상 1 — 체크섬 실패 문장은 행을 만들지 않고 요약에 집계 |
| `carried_fields` | str | 이 행에서 `source_sentence`가 아닌 **이전** 문장에서 이어받은 필드 이름, `\|` 구분 |

### 행 주기와 이어받은 필드

- GGA마다 1행, RMC마다 1행 → **1 Hz fix에서 초당 2행**입니다.
- 생성 문장에 없는 필드는 직전 행에서 이어받습니다(RMC 행에는 고도·fix 품질, GGA 행에는 속도·방위).
  `source_sentence`는 생성 문장을, `carried_fields`는 **새 값이 아닌** 필드를 정확히 알려줍니다.

이 차이는 생각보다 중요합니다. 사용한 수신기는 매 epoch마다 `RMC`를 **먼저** 보내므로,
RMC 행의 `altitude_m`, `fix_quality`, `satellite_count`는 *직전 초*의 GGA 값입니다.
그래서 fix가 바뀌는 순간에는 행 내부가 모순될 수 있습니다. 예를 들어 fix를 잡은 직후 첫 RMC 행은 `fix_valid=1`과 `fix_quality=0`을 함께 가집니다.
값을 믿기 전에 `carried_fields`를 확인하거나, GGA 행만 사용하십시오.

### 예상되는 패턴

한 묶음의 두 행은 약 5 ms 간격으로 도착하고, 이후 약 995 ms 동안 조용합니다.
따라서 타임스탬프 감사는 GPS에 대해 `gaps`와 1000 ms 근처의 지터 p95를 보고합니다. 이는 수신기의 1 Hz 묶음 송신 특성이며 결함이 아닙니다.

`raw_nmea.log`에는 유효 여부와 관계없이 수신한 모든 문장이 두 호스트 타임스탬프와 함께 남습니다. 파서는 언제든 이 파일로 다시 돌릴 수 있습니다.

## 4. `imu/imu.csv`

원시 카운트와 공학 단위를 나란히 저장하고, 센서 좌표계와 차량 좌표계를 분리해 둡니다. 어떤 값도 제자리에서 덮어쓰지 않습니다.

| 그룹 | 열 |
|---|---|
| 식별 | `host_monotonic_ns`, `host_wall_time_ns`, `sensor_sequence` |
| 원시 카운트 | `raw_acc_{x,y,z}`, `raw_gyro_{x,y,z}`, `raw_mag_{x,y,z}`, `raw_{roll,pitch,yaw}`, `raw_q{0,1,2,3}` |
| 센서 좌표계, SI | `acc_{x,y,z}_mps2`, `gyro_{x,y,z}_radps`, `mag_{x,y,z}`, `{roll,pitch,yaw}_rad`, `quaternion_{w,x,y,z}` |
| 차량 좌표계 | `veh_acc_{x,y,z}_mps2`, `veh_gyro_{x,y,z}_radps` |
| 기타 | `temperature_c`, `packet_type`, `packets_in_sample`, `checksum_valid`, `sample_valid` |

### 단위 변환

WIT 표준 프로토콜 기준이며, 풀스케일 값은 `configs/daq.yaml`에서 읽습니다.

```text
acc   = raw/32768 * 16 g     * 9.80665   -> m/s²
gyro  = raw/32768 * 2000 °/s * π/180     -> rad/s
angle = raw/32768 * 180°     * π/180     -> rad
quat  = raw/32768
temp  = raw/100                          -> °C
mag   = 원시 카운트 (미보정, 장치가 스케일 값을 제공하지 않음)
```

### 차량 좌표계

- **차량 좌표계는 `+x 전방, +y 좌측, +z 위`, 오른손 좌표계**입니다.
- `veh_* = R · (sensor − bias)`이며 `R`은 `imu.mounting.rotation_matrix`입니다.
- 기본 설정의 `R`은 **단위행렬**(장착 각도 미측정)이므로 `veh_*`는 센서 좌표계 값과 같습니다.
  두 값을 모두 저장하므로 재수집 없이 나중에 변환을 적용할 수 있습니다.

### 기타

- `sample_valid`는 가속도와 자이로가 모두 있고 체크섬이 정상일 때만 1입니다.
- `packet_type`은 샘플에 기여한 WIT 패킷 목록입니다. 예: `acc+angle+gyro+mag`
- `raw_*.bin`은 가공하지 않은 바이트 스트림입니다. 조각을 이름 순서로 이어 붙이면 회선에 도착한 바이트를 그대로 재현합니다.

## 5. `camera/frame_timestamps.csv`

디스크에 쓰지 않은 프레임까지 포함해 **캡처한** 프레임마다 1행입니다.

| 열 | 설명 |
|---|---|
| `frame_id` | 1부터 시작, 캡처한 모든 프레임을 셈 |
| `capture_monotonic_ns` | `read()`가 반환한 순간의 시각 — **컨테이너 타임스탬프 대신 이 값을 사용** |
| `host_wall_time_ns` | |
| `segment_id` | 이 프레임이 들어 있는 `driver_NNN.mkv` 번호, 기록하지 않았으면 `-1` |
| `frame_index_in_segment` | 조각 내 위치, 기록하지 않았으면 `-1` |
| `recorded` | 디스크에 썼으면 1, 솎아내기(decimation)로 버렸으면 0 |
| `capture_valid` | 버퍼가 정상 JPEG(SOI + EOI 존재)이면 1 |
| `dropped_since_previous` | 캡처 간격으로 추정한 누락 프레임 수 |
| `jpeg_bytes` | 패딩 제거 후 크기 |

- `record_decimation: 4`이면 4행마다 1행이 `recorded=1`입니다. 기록된 프레임을 영상에서 찾으려면
  `recorded==1`로 거른 뒤 `(segment_id, frame_index_in_segment)`로 정확히 지정합니다.
- **컨테이너 타임스탬프는 명목값입니다.** ffmpeg에는 고정 명목 주기(`capture_fps / record_decimation`)를 주며, 실제 시간은 이 CSV에만 있습니다.
- `dropped_since_previous`는 *추정값*입니다(간격 > 기대값 × 1.5). 이 경로에서는 V4L2가 누락을 알려주지 않으므로 개수가 아닌 지표로 보십시오.

### 알려진 한계 — 노출 시각이 아닌 디큐 시각

`capture_monotonic_ns`는 `read()`가 반환한 순간에 찍힙니다. OpenCV는 V4L2 버퍼 타임스탬프를 노출하지 않고,
드라이버는 최대 `CAP_PROP_BUFFERSIZE`(4)개 프레임을 큐에 쌓습니다. 캡처 스레드가 잠시 지연되면
쌓인 프레임이 연달아 디큐되어, 실제 노출 시각보다 최대 지연 시간만큼 *늦은* 타임스탬프를 받습니다.
데이터에서는 긴 간격 뒤에 1 ms 미만 간격이 두세 번 이어지는 형태로 뚜렷하게 나타납니다.

```text
gap 289.0 ms, then intervals (ms): 0.228, 0.095, 0.094, 18.793 ...
```

- 이 경우 `dropped_since_previous`는 공백 전체를 묶음의 첫 프레임에 귀속시킵니다.
- 기록용 writer 스레드를 분리해 발생 빈도를 줄였지만(분리 전에는 세션당 약 20프레임), 이 캡처 API 고유의 성질이라 완전히 없앨 수는 없습니다.
- **1 ms 미만 간격이 보이면 그 묶음의 타임스탬프는 상한값으로 취급하십시오.**
  그 밖의 `segment_id`, `frame_index_in_segment`, 순서, 단조성은 모두 정확합니다.

## 6. `pose/features.csv`

자세 추정 프로세스가 돌려준 프레임마다 1행이며, 사람이 없는 프레임도 포함합니다.

| 그룹 | 열 |
|---|---|
| 식별 | `host_monotonic_ns`, `host_wall_time_ns`, `frame_id`, `capture_monotonic_ns` |
| 기하 | `shoulder_center_{x,y}`, `hip_center_{x,y}`, `head_center_{x,y}`, `shoulder_width`, `torso_angle_deg` |
| 중립 대비 | `shoulder_shift_{x,y}`, `hip_shift_{x,y}`, `head_shift_{x,y}`, `shoulder_shift_px`, `head_shift_px` |
| 파생 | `lean_score`, `lean_velocity` |
| 품질 | `pose_confidence`, `landmark_valid_mask`, `landmarks_visible`, `pose_processing_latency_ms`, `neutral_valid`, `vision_valid` |

| 항목 | 정의 |
|---|---|
| 좌표 | **MediaPipe 정규화 좌표**(이미지의 0–1). `*_px`는 `pose.input_width/height`를 곱한 값 |
| `head_center` | 코 + 양쪽 귀가 보이면 그 평균, 아니면 코만 |
| `torso_angle_deg` | `atan2(shoulder_x − hip_x, hip_y − shoulder_y)`를 도 단위로. **0 = 똑바로**, 양수 = 이미지 +x 방향으로 기울어짐 |
| `lean_score` | `shoulder_shift_x / neutral.shoulder_width` — 어깨 너비로 정규화해 운전자·카메라 거리와 무관하게 비교 가능 |
| `lean_velocity` | 호스트 시계 기준 d(`lean_score`)/dt |
| `landmark_valid_mask` | 추적하는 랜드마크마다 한 글자, MSB 우선, MediaPipe 인덱스 오름차순(`0,7,8,11,12,13,14,15,16,23,24`). `1` = visibility ≥ 0.5 |
| `vision_valid` | 양 어깨와 양 골반의 visibility가 모두 ≥ 0.5 |

**모든 `*_shift_*`와 `lean_*` 열은 중립 자세 보정이 성공하기 전까지 null입니다.**
성공 여부는 `neutral_valid`, 실패 이유는 `session_summary.json → workers.pose.neutral.reason`에 있습니다.

### `pose/landmarks.jsonl`

**랜드마크가 검출된** 프레임마다 객체 1개입니다(화면에 사람이 없으면 비어 있음).

```json
{"frame_id": 1234, "capture_monotonic_ns": …, "host_wall_time_ns": …,
 "latency_ms": 72.4,
 "landmarks": [{"i":11,"n":"left_shoulder","x":0.54,"y":0.48,"z":-0.10,"v":1.0,"p":1.0}, …]}
```

`i` MediaPipe 인덱스, `n` 이름, `v` visibility, `p` presence.

## 7. `health/health.csv`

1 Hz로 기록합니다.

| 분류 | 항목 |
|---|---|
| 시스템 | `state`, `cpu_percent`, `cpu_temp_c`, `load_1m`, 메모리, `disk_free_gb` |
| 디스크 | `disk_write_latency_ms` — 실제 64 KiB write+fsync, 최대 30 s마다 1회 측정 |
| 작업자 | 생존 여부, 수신율, 최신 데이터 경과 시간, 체크섬 오류 |
| 카메라 | FPS, 기록 수, 누락 수 |
| 자세 | FPS, 지연, 유효 비율 |
| 기타 | 큐 깊이, 누적 경고·오류 수 |

## 8. `events.jsonl`

```text
{host_monotonic_ns, host_wall_time_ns, level, kind, source, …}
level ∈ {info, warning, error}
```

주요 `kind`: `session_started`, `state_changed`, `worker_started/stopped/failed`, `device_lost`,
`recorder_died`, `sensor_stale`, `disk_space_low/exhausted`, `storage_root_changed`,
`neutral_calibrated`, `neutral_calibration_failed`.

## 9. `metadata.json`

- OS·커널·아키텍처·Pi 모델, Python과 패키지 버전, git commit과 수정 여부(dirty)
- 시계 소스, 저장 경로와 마운트 정보(장치, 파일시스템, UUID, 옵션)
- USB 식별 정보와 실제 baud·포맷을 포함한 전체 장치 목록
- 카메라 설치 위치(`driver_side_a_pillar_upper`), 차량 좌표계 정의와 IMU 장착 변환
- 사용한 설정 파일 전체 사본

## 10. `session_summary.json`

길이, 최종 상태, 작업자별 카운터, 저장소 상태, 이벤트 수, 경로 통계와 스트림별 **타임스탬프 감사**를 담습니다.

```json
"imu": {"count": 1535, "monotonic": true, "backward_steps": 0,
        "duplicates": 0, "gaps": 0, "max_gap_s": 0.6,
        "measured_hz": 10.05, "jitter_ms_p50": 0.81, "jitter_ms_p95": 3.22}
```

| 필드 | 의미 |
|---|---|
| `duplicates` | 0보다 크면 여러 샘플이 같은 호스트 타임스탬프를 공유 — 시리얼 읽기 묶임 결함을 잡아내는 지표. 분석 전에 확인 |
| `gaps` | 최소 1개 샘플 누락을 뜻하는 간격 수 (임계값 `중앙값 × 1.75`, `gap_threshold_s`로 보고) |
| `large_gaps` | 임계값 `× 3` 기준의 간격 수 |
| `estimated_missed_samples` | 간격들로 설명되는 누락 샘플 수 합계 |

GPS 항목은 **GGA 행만** 감사합니다(epoch당 1행). 1 Hz epoch의 두 행을 모두 감사하면 수신기가 2 Hz에 약 900 ms 지터로 보이기 때문입니다.
필터링하지 않은 결과는 `gps_all_rows`에 함께 남습니다.

## 11. 저장소(git)에 넣지 않는 것

세션 데이터는 저장소에 들어가지 않습니다. 기록 데이터는 SSD의 `OASIS_DAQ/`(또는 대체 경로 `oasis-daq-fallback/`)에만 남고, 운영자가 별도로 옮깁니다.
저장소의 `.gitignore`도 영상(`*.mkv`), 원시 바이트(`*.bin`), 로그(`*.log`, `raw_nmea.log` 포함)를 제외합니다.
