[← Raspberry Pi 센서 수집기](../README.md)

# 알려진 제약과 해결 상태

> 수집기(OASIS DAQ v0.1)의 코드 결함이 아닌 제약 — 하드웨어, 설치 환경, 물리적 시험이 필요한 항목 — 과 각 항목의 현재 상태, 해결 방법을 정리합니다.
> 시험 근거는 [TEST_RESULTS](TEST_RESULTS.md)에 있습니다.

---

## 요약

| # | 항목 | 분류 | 상태 |
|---|---|---|---|
| 1 | [GPS 측위](#1-gps-측위--실차에서-해결) | 환경 | **해결** — 실차 주행에서 측위 확인 |
| 2 | [실시간 중립 자세 보정](#2-실시간-중립-자세-보정) | 알고리즘 | **우회** — 자세 검출은 동작, 중립 기준은 오프라인 계산 |
| 3 | [IMU 10 Hz](#3-imu-출력-10-hz) | 설계 제약 | 유지 — 공장 설정 그대로 사용 |
| 4 | [외장 SSD 마운트 시점](#4-외장-ssd-마운트-시점) | 운영 | **소프트웨어로 완화** |
| 5 | [전원 차단 내구성](#5-전원-차단-내구성--부분-검증) | 검증 | 부분 검증 (SIGKILL 통과, 실제 전원 차단 미시험) |
| 6 | [IMU 장착 회전 행렬](#6-imu-장착-회전-행렬--항등-행렬) | 보정 | 항등 행렬 — 원시·차량 좌표 모두 저장, 후처리에서 적용 가능 |
| 7 | [5 V / 3 A 전원 용량](#7-5-v--3-a-전원-용량-부족) | 하드웨어 | 미해결 — 5 V / 5 A 전원 권장 |
| 8 | [기타 미검증 항목](#8-기타-미검증-항목) | — | GPIO 버튼 · Hailo · GPS/IMU 의도적 분리 |

---

## 1. GPS 측위 — 실차에서 해결

**상태:** 해결. 실내 벤치에서는 하늘이 보이지 않아 측위하지 못했지만, 실차 주행에서 주 세션 기준 유효 측위 1,315건과 실제 `route.geojson`을 얻었습니다.
주행 데이터 처리는 [2. 데이터 처리](../../../2_Data_Processing/README.md)를 참고하십시오.

벤치에서도 수신기 자체는 정상입니다. u-blox 7이 NMEA를 **체크섬 실패 0**으로 계속 송출하고 `ANTSTATUS=OK`, `GPGSV` 기준 위성 3–4개(SNR 22–23)를 추적합니다.

```text
$GPGGA,,,,,,0,00,99.99,,,,,,*48      <- 측위 품질 0, 사용 위성 0
$GPRMC,,V,,,,,,,,,,N*53              <- 상태 V (무효)
```

측위 여부 확인 절차:

```bash
oasis-daqctl check --probe --probe-seconds 60     # fix=yes 확인 (냉시동 측위 1–15분)
oasis-daqctl start --notes "gps fix validation"
oasis-daqctl stop
python3 -c "import json;print(json.load(open('<session>/route_summary.json')))"
```

---

## 2. 실시간 중립 자세 보정

**상태:** 우회. 실차 주행에서 착석자 검출은 동작했지만(`vision_valid` 99.9 %), 실시간 중립 보정은 세션 내내 고정되지 않았습니다(`neutral_valid` = 0).
따라서 실차 데이터의 중립 기준은 오프라인에서 계산합니다.

| 항목 | 상태 |
|---|---|
| 흑백→RGB 입력 경로 | 샘플 사진으로 검증 — 랜드마크 33개, 컬러 경로 대비 좌표 차이 <0.01 |
| 실차 착석자 검출 | 동작 (`vision_valid` 99.9 %) |
| 실시간 중립 보정 | 고정되지 않음 (`neutral_valid` = 0) → 오프라인 계산으로 대체 |

중립 보정 조건 (`configs/daq.yaml` → `pose.neutral_calibration`):

| 키 | 값 | 의미 |
|---|---|---|
| `start_after_s` | 5.0 | 세션 시작 후 대기 시간 |
| `window_s` | 10.0 | 안정 구간 길이 |
| `min_valid_frames` | 30 | 구간 내 최소 유효 프레임 |
| `max_landmark_stddev_px` | 12.0 | 랜드마크 흔들림 허용치 |

조건을 만족하지 못하면 `only N valid frames in window, need 30` 같은 사유와 함께 `neutral_calibration_failed` 경고를 남기며, 기록은 계속됩니다.
최종 제어기는 이 실시간 보정 대신 세션 전에 받는 RTMPose 중립 파일을 사용합니다([3_RL_Control/deploy](../../../3_RL_Control/deploy/README.md)).

---

## 3. IMU 출력 10 Hz

**상태:** 설계 제약. v0.1은 확인된 공장 설정을 그대로 기록하며 센서 설정을 바꾸지 않습니다.

| 항목 | 값 |
|---|---|
| 실측 설정 | **9600 baud, 10 Hz** (`acc` · `gyro` · `angle` · `mag` 각각), 체크섬 실패 0 |
| 대역폭 | 샘플당 11바이트 프레임 4개 → 약 440 B/s, 9600 baud 회선은 약 21 Hz에서 포화 |
| 적합한 용도 | 자세, 차량의 큰 동역학 |
| 부적합한 용도 | **급격한 과도 응답** (포트홀 충격, 급제동 시작, 충돌 감지) |

10 Hz 이상이 필요하면 **센서의 baud 자체를 바꿔야** 합니다. WIT 설정 쓰기가 필요하므로 다음 순서로 신중히 진행합니다.

1. 현재 설정을 먼저 기록합니다.
2. WIT 잠금 해제 + baud + 출력 주기 명령을 보냅니다 (WITMOTION 공식 `WitStandardProtocol_JY901` 참고).
3. `oasis-daqctl check --probe --probe-imu-baud`가 새 baud를 `frame_ratio > 0.99`로 선택하는지 확인합니다.
4. `configs/daq.yaml`의 `imu.baud`를 갱신하고 `imu.expected_hz`(타임스탬프 감사 기준 주기)를 설정합니다.
5. 변경 전후 설정을 세션 메타데이터에 남깁니다.

`imu.baud_candidates`가 이미 설정에 있고 탐색은 읽기 전용이므로, 잘못 설정해도 센서 전원을 껐다 켜면 복구할 수 있습니다.

---

## 4. 외장 SSD 마운트 시점

**상태:** 소프트웨어로 완화. 시스템 수준 수정(`/etc/fstab`)은 적용하지 않았습니다.

외장 SSD(Samsung T7)는 **데스크톱 로그인 후 udisks2가** 마운트합니다. `oasis-daq.service`는 그보다 훨씬 이른 부팅 시점에 시작합니다.
실제로 재부팅 후 데몬이 SSD 없이 기동해 로컬 대체 경로를 선택한 사례가 있습니다([TEST_RESULTS §9](TEST_RESULTS.md#9-실제-하드웨어-장애-대응)).

| 대책 | 내용 |
|---|---|
| 세션 시작마다 저장 경로 재확인 | `start_session()` → `_refresh_storage_root`. SSD가 나타난 뒤 START를 누르면 SSD에 기록하고 `storage_root_changed` 이벤트를 남김 |
| 마운트 지점 검사 | `storage.require_mountpoint: true` — 남아 있는 빈 폴더를 마운트된 SSD로 오인하지 않음 |
| 사용 중인 경로 표시 | `oasis-daqctl status`와 대시보드에 항상 표시 |
| 자동 기록 비활성 | `session.autostart_recording: false` — 부팅 직후 세션이 microSD 대체 경로에 기록되는 것을 방지 |

**잔여 위험:** `autostart_recording: true`로 바꾸면 부팅 시 시작된 세션이 microSD 대체 경로에 기록될 수 있습니다.

근본 해결은 fstab 등록입니다(변경 전 반드시 백업).

```bash
sudo cp /etc/fstab /etc/fstab.bak
echo 'UUID=<SSD UUID> <SSD 마운트 경로> exfat defaults,nofail,uid=1000,gid=1000,x-systemd.device-timeout=10 0 0' \
  | sudo tee -a /etc/fstab
sudo mount -a                # 재부팅 전에 반드시 성공해야 함
findmnt <SSD 마운트 경로>
# 이후 systemd/oasis-daq.service의 RequiresMountsFor= 주석 해제
```

`nofail`과 장치 타임아웃은 필수입니다. 없으면 SSD가 빠졌을 때 부팅이 멈춥니다.

---

## 5. 전원 차단 내구성 — 부분 검증

**상태:** 부분 검증. 남은 시험은 전원을 물리적으로 차단해야 합니다.

`flush()`는 커널 페이지 캐시까지만 도달하므로, 기록 중 SSD가 분리되면 데이터가 남지 않을 수 있습니다(실제 사례: [TEST_RESULTS §5](TEST_RESULTS.md#5-내구성)). 현재 대책은 다음과 같습니다.

| 대책 | 설정 |
|---|---|
| 모든 기록기 주기적 `fsync` | `storage.fsync_interval_s: 15.0` |
| 완료된 영상 조각 즉시 `fsync` | ffmpeg가 조각을 마칠 때마다 |
| 영상 조각 길이 60초 | `camera.segment_seconds: 60` — 최악의 영상 손실 구간 |

| 시험 | 결과 |
|---|---|
| 데몬 SIGKILL | 통과 — CSV는 종료 1–5초 전까지, 진행 중 조각 포함 영상 모두 재생 가능 |
| 32분 31초 내구 시험 | 통과 — 오류 0, 정상 종료 |
| **실제 전원 차단** | **미시험** — 페이지 캐시 우회를 증명할 유일한 방법 |

시험 절차:

```bash
oasis-daqctl start --notes "power loss test"
sleep 180                      # 조각 2–3개 완료 대기
# Pi 전원을 물리적으로 제거 (shutdown 하지 않음), 다시 켠 뒤:
oasis-daqctl summarize <SSD>/OASIS_DAQ/sessions/<session>
```

기대 결과: `INCOMPLETE` 존재, 모든 CSV에 차단 약 15초 전까지 데이터, **완료된** `driver_*.mkv`는 모두 재생 가능.
진행 중이던 조각은 손실되거나 잘릴 수 있으며, 이것이 문서화된 최악의 경우입니다.

---

## 6. IMU 장착 회전 행렬 — 항등 행렬

**상태:** 미측정. `configs/daq.yaml`의 `imu.mounting.rotation_matrix`는 항등 행렬이며,
`"identity -- sensor axes not yet measured against vehicle axes"`로 명시되어 있습니다.

| 항목 | 내용 |
|---|---|
| 현재 영향 | `veh_*` 열이 센서 좌표 열과 같음 |
| 데이터 손실 | 없음 — 모든 행에 센서 좌표와 차량 좌표를 별도로 저장, 후처리에서 회전 적용 가능 |
| 검증 | 회전 행렬은 직교성과 `det = +1`을 검사 (`det = −1` 반사 행렬은 좌우를 조용히 뒤집으므로 거부) |
| 카메라 위치 | `driver_side_a_pillar_upper`로 기록 — 물리적으로 별도 검증하지 않음 |

해결 방법: IMU를 최종 방향으로 고정하고, 차량이 수평·정지 상태일 때 `imu.csv`에서 중력 벡터를 차량 `-z`로, 전진 가속을 `+x`로 보내는 회전을 구해
`configs/daq.yaml`에 기록합니다. 보정 내용은 `configs/imu_calibration.yaml`(예시: `imu_calibration.yaml.example`)에 남깁니다.

---

## 7. 5 V / 3 A 전원 용량 부족

**상태:** 미해결. 차량 사용 전에 전원 교체를 권장합니다.

Raspberry Pi 5는 **5 V / 5 A**를 기대합니다. **5 V / 3 A (15 W) PD 전원**에서는 펌웨어가 **USB 총 전류를 600 mA로 제한**하는데,
USB 장치는 GPS, IMU, 카메라, Samsung T7 네 개이고 T7은 연속 쓰기 중 혼자서도 600 mA를 넘을 수 있습니다.

시험 중 원인 불명의 USB 이탈이 두 번 발생했습니다.

| 이벤트 | 결과 |
|---|---|
| SSD 분리 후 재부팅 | 약 10분 수집분 손실 → `fsync` 부재 결함을 드러냄 (수정 완료) |
| 카메라 분리, 1초 뒤 재연결 | 세션이 ERROR로 정상 종료, 영상 조각 11개 모두 재생 가능 |

`vcgencmd get_throttled`는 내내 `0x0`이었지만, 이 값은 Pi **입력 레일**만 보고하며 USB 포트 전류 예산은 반영하지 않으므로 원인에서 배제할 수 없습니다.
두 이탈 모두 소프트웨어가 원인이 아니고 수집기는 올바르게 처리했지만, 주행 중 이탈은 진행 중인 데이터를 잃게 하며 차량 전원에는 시동 시 과도 전압도 있습니다.

해결 방법 (우선순위 순):

1. **5 V / 5 A (25 W) USB-C PD 전원** 사용 (공식 Pi 5 전원 또는 동급) — 근본 해결
2. 불가하면 SSD를 **유전원 USB 허브**에 연결해 Pi가 아닌 허브에서 전류를 받게 함
3. (1)을 적용한 뒤에만 `/boot/firmware/config.txt`에 `usb_max_current_enable=1`을 설정해 600 mA 제한 해제. **15 W 전원에서는 설정 금지** — 전류는 늘지 않고 보호만 사라짐

세션 전후 확인:

```bash
vcgencmd get_throttled       # 0x0 = 정상
                             # bit 0  = 현재 저전압
                             # bit 16 = 부팅 이후 저전압 발생
```

SSD는 **USB 2.0** 포트(480 Mbps)에 연결되어 있습니다. 수집기가 만드는 약 4.4 MB/s에는 충분하며, 다시 꽂는 것 자체가 이탈 위험이므로 그대로 둡니다.

---

## 8. 기타 미검증 항목

| 항목 | 상태 |
|---|---|
| **GPIO START/STOP 버튼** | `configs/daq.yaml`에 `gpio_button` 설정 항목만 있으며 `enabled: false`, `pin: null`. 실제 배선에서 핀이 확정되지 않았으므로 활성화하지 않음 |
| **Hailo 가속** | 수집기는 MediaPipe를 CPU(XNNPACK)로 실행하며 v0.1 목표 성능을 만족하므로 Hailo에 의존하지 않음. 최종 제어기의 RTMPose + Hailo 경로는 [3_RL_Control/deploy](../../../3_RL_Control/deploy/README.md) |
| **기록 중 센서 분리** | 카메라 분리는 실제로 발생했고 설계대로 처리됨 — 50회 연속 읽기 실패 후 `device_lost`, ERROR로 정상 종료, 영상 조각 11개 모두 재생 가능, GPS · IMU · 자세는 **오류 0**으로 계속 동작, udev가 `/dev/oasis_camera` 재생성. **GPS · IMU의 의도적 분리 시험은 수행하지 않음** |
