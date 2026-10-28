[← Raspberry Pi 센서 수집기](../README.md)

# 운영 가이드

> 수집기의 설치, 일상 사용, 대시보드·API, 설정, 결과 확인, 문제 해결, 복구 절차를 정리합니다.
> 경로 표기: `<repo>` = 저장소 루트, `<venv>` = 수집기용 Python 3.11 가상환경, `<ssd>` = 외장 SSD 마운트 지점.

---

## 1. 설치 (최초 1회)

```bash
# 1. Python 환경 (수집기는 Python 3.11 가상환경에서 동작)
<venv>/bin/python -m pip install -e "<repo>/1_Sensor_Acquisition/raspberry_pi"

# 2. 자세 추정 모델 (약 6 MB, git에 넣지 않음)
bash "1_Sensor_Acquisition/raspberry_pi/scripts/fetch_pose_model.sh" lite

# 3. udev 규칙 + systemd 유닛 (여러 번 실행해도 안전, 교체하는 파일은 백업)
sudo bash "1_Sensor_Acquisition/raspberry_pi/scripts/install.sh"

# 4. 확인
oasis-daqctl check --probe --probe-imu-baud
```

`install.sh`는 `/etc/fstab`을 건드리지 않고, SSD를 마운트·포맷·파티션하지 않습니다.

### 경로에 공백이 있는 경우 (systemd)

저장소를 공백이 포함된 경로에 둘 경우 systemd 유닛 파일에서 다음 세 규칙을 지켜야 합니다.
`systemd/oasis-daq.service`는 이미 이 규칙대로 작성되어 있으므로 "단순화"하지 마십시오.

| 위치 | 규칙 | 지키지 않으면 |
|---|---|---|
| `ExecStart=` | 경로를 **반드시 따옴표로 감쌈** | systemd가 공백에서 잘라 두 개의 인자로 넘김 |
| `WorkingDirectory=` | 경로를 따옴표로 **감싸지 않음** | 줄 나머지를 그대로 받으므로 따옴표가 경로 일부가 됨: `path is not absolute` |
| `Documentation=` | 공백을 `%%20`으로 퍼센트 인코딩 | 공백 구분 URL 목록이며, `%` 하나는 systemd 지정자로 해석됨: `Invalid slot` |

셸에서는 항상 따옴표로 감쌉니다.

```bash
pip install -e "1_Sensor_Acquisition/raspberry_pi"
pytest "1_Sensor_Acquisition/raspberry_pi/tests/" -q
```

유닛 파일을 고친 뒤에는 `systemd-analyze verify /etc/systemd/system/oasis-daq.service`로 확인합니다. 문제가 없으면 아무것도 출력하지 않습니다.

## 2. 일상 사용

```bash
sudo systemctl start oasis-daq      # (부팅 시 자동 시작으로 등록되어 있음)
oasis-daqctl status                 # READY 상태와 저장 경로 확인
oasis-daqctl start --notes "run1"
#   ... 주행 ...
oasis-daqctl stop
oasis-daqctl summarize <ssd>/OASIS_DAQ/sessions/<date>/<session>
```

**세션마다 시작 전에 저장 경로를 확인하십시오.** `oasis-daqctl status`가 `~/oasis-daq-fallback`을 보여주면,
데몬이 저장소를 결정할 때 외장 SSD가 마운트되어 있지 않았던 것입니다.
Supervisor는 `start` 시점에 저장소를 다시 결정하므로, SSD를 마운트(파일 관리자를 열거나 `udisksctl mount -b /dev/sda1`)한 뒤
START를 다시 누르면 세션이 SSD에 기록됩니다. 자세한 내용은 [LIMITATIONS](LIMITATIONS.md)의 "외장 SSD 마운트 시점" 항목을 참고하십시오.

## 3. 노트북 대시보드

브라우저에서 `http://<pi-ip>:8080` 또는 mDNS로 `http://<hostname>.local:8080`에 접속합니다.

| 영역 | 표시 내용 |
|---|---|
| 세션 | 상태, 경과 시간, START/STOP |
| 카메라 | 실시간 미리보기 + MediaPipe 골격, 자세 신뢰도, 기울기 지표 |
| GPS | fix·위성 수·HDOP·위치·속도·방위 |
| IMU | 가속도·자이로·자세각, g-force |
| 상태 | 센서별 수신율, 카메라 FPS·누락, SSD 여유 공간과 남은 녹화 가능 시간, 최근 이벤트 |
| 지도 | 실시간 주행 궤적 |

- **노트북은 선택 사항입니다.** 브라우저를 닫거나 Wi-Fi가 끊기거나 포트가 막혀도 진행 중인 세션에는 영향이 없습니다([TEST_RESULTS](TEST_RESULTS.md)).
  원본 데이터는 항상 SSD로만 가고 네트워크로 나가지 않습니다.
- 지도 타일(OpenStreetMap)은 **노트북**의 인터넷 연결로 받으며 Pi가 중계하지 않습니다.
  타일이 없으면 빈 화면 대신 단순 격자 위에 궤적을 그립니다.

### API

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET | `/api/status` | 전체 상태, 모든 작업자, 저장소 |
| GET | `/api/health` | 상태 샘플 1개 (CPU, 온도, 메모리, 디스크 지연) |
| GET | `/api/latest` | 최신 GPS 행, IMU 행, 자세 결과 |
| GET | `/api/preview.jpg` | 축소된 미리보기 프레임 1장 |
| GET | `/api/stream.mjpg` | multipart MJPEG 미리보기 스트림 |
| GET | `/api/route.geojson` | 현재까지의 실시간 궤적 |
| POST | `/api/session/start` | `{"notes": "...", "suffix": "..."}` |
| POST | `/api/session/stop` | `{"reason": "..."}` |

**인증이 없습니다.** 8080 포트는 신뢰할 수 있는 네트워크에만 노출하십시오.

## 4. 명령줄 도구

```bash
oasis-daqctl check [--probe] [--probe-seconds N] [--probe-imu-baud] [--json FILE]
oasis-daqctl status [--json]
oasis-daqctl start [--notes TEXT] [--suffix TEXT]
oasis-daqctl stop [--reason TEXT]
oasis-daqctl summarize <session-dir> [--rebuild]
oasis-daqctl daemon [--record] [--duration SECONDS]      # systemd가 사용
```

- `start`/`stop`/`status`는 실행 중인 데몬과 HTTP API로 통신합니다. 장치 소유자는 데몬 하나뿐이어야 하며, 소유자가 둘이면 세션이 손상됩니다.
- `daemon --duration`은 벤치·장시간 시험용입니다.

## 5. 설정

설정 파일: `1_Sensor_Acquisition/raspberry_pi/configs/daq.yaml`

| 키 | 기본값 | 설명 |
|---|---|---|
| `storage.roots` | `/media/<user>/T7/OASIS_DAQ`, `/mnt/T/OASIS_DAQ` | 마운트되어 있고 쓰기 가능한 첫 경로를 사용 |
| `storage.min_free_gb_start` | 20 | 이보다 적으면 시작 거부 |
| `storage.min_free_gb_stop` | 5 | 세션을 정상 종료하고 ERROR로 전환 |
| `storage.fsync_interval_s` | 15 | 전원 차단 시 데이터 손실 범위를 제한 |
| `session.autostart_recording` | **false** | 아래 경고 참고 |
| `camera.capture_fps` | 100 | 1280x800에서 장치가 100/120만 제공 |
| `camera.record_decimation` | 4 | 25 fps 기록, ≈3.9 GB/h. `1`이면 전부 기록 (≈16 GB/h) |
| `camera.segment_seconds` | 60 | 최악의 영상 손실 범위 |
| `pose.target_fps` | 10 | 실측 상한은 약 13 fps |
| `dashboard.port` | 8080 | |
| `imu.mounting.rotation_matrix` | 단위행렬 | **IMU를 차량에 장착한 뒤 측정해서 넣어야 함** |

**`autostart_recording: true`로 바꾸지 마십시오.** SSD는 로그인 후 udisks2가 마운트하므로, 부팅 직후 시작한 세션은 microSD 대체 경로에 기록됩니다.

설정을 바꾼 뒤에는 `sudo systemctl restart oasis-daq`로 재시작합니다.

## 6. 결과 확인

```bash
oasis-daqctl summarize <session-dir>
```

다음 순서로 확인합니다.

1. **`INCOMPLETE`가 있는가?** 있으면 중단된 세션입니다. 모든 스트림의 마지막 약 15 s와 진행 중이던 영상 조각이 없을 수 있습니다.
2. **`timestamp_quality`** — `monotonic`은 true, `duplicates`는 0이어야 합니다. 중복이 있으면 여러 샘플이 같은 호스트 타임스탬프를 공유해 정밀 정렬이 불가능합니다.
3. **작업자 카운터** — `errors`, `dropped`, `frames_bad_checksum`, `estimated_drops`.
4. **`workers.pose`의 `neutral.reason`** — 중립 보정이 실패했다면 모든 `*_shift_*`, `lean_*` 열이 null입니다.

**GPS의 `gaps`와 약 1000 ms 지터 p95는 정상입니다.** GGA와 RMC가 초당 한 번 함께 도착하는 수신기 특성이며 결함이 아닙니다.

## 7. 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `check`에서 장치 없음 | 연결 안 됨, 또는 udev 규칙 미설치 | `ls -l /dev/oasis_*`, `install.sh` 재실행, `bash scripts/inspect_hardware.sh` |
| 저장 경로가 대체 경로 | 데몬 시작 시 SSD 미마운트 | 마운트 후 `oasis-daqctl start` (저장소 재결정) |
| `start` 거부, "GB free" | `min_free_gb_start` 미만 | SSD 공간 확보, 또는 의도적으로 임계값 하향 |
| 자세 작업자 없음, `pose_disabled` 이벤트 | 모델 파일 없음 | `bash scripts/fetch_pose_model.sh lite` |
| 카메라 FPS가 몇 초간 낮음 | 일시적 CPU 부하 | 저절로 회복. 계속 낮으면 실제 자원 경합 |
| 세션이 ERROR로 전환 | 기록기 종료, 디스크 소진, 카메라 분리 | `events.jsonl` 확인 후 `journalctl -u oasis-daq` |
| 교체 후 IMU baud 불명 | 다른 펌웨어 | `oasis-daqctl check --probe --probe-imu-baud` (읽기 전용) |

## 8. 전원 — 반드시 확인

Pi는 **5 V / 3 A (15 W) PD 전원**에 연결되어 있습니다. Raspberry Pi 5의 정격은 5 V / 5 A이며,
15 W 전원에서는 펌웨어가 **USB 총 전류를 600 mA로 제한**합니다. Samsung T7 하나만으로도 지속 쓰기 중 이를 넘을 수 있고,
USB 장치는 GPS·IMU·카메라·SSD 네 개입니다. 장시간 시험 중 원인 불명의 SSD 분리와 재부팅이 관찰되었으며, 이는 전력 예산 초과와 일치합니다.

- `vcgencmd get_throttled`를 확인합니다. `0x0`이 정상이며, bit 0 = 현재 저전압, bit 16 = 부팅 후 저전압 발생.
- 세션 중에는 USB 장치를 추가하지 않습니다.
- 장거리 주행 전에는 5 V / 5 A 전원을 쓰거나 SSD를 자체 전원 허브로 연결합니다.
- SSD는 **USB 2.0** 포트(480 Mbps)에 연결되어 있습니다. 수집기가 만드는 4.4 MB/s에는 충분하며, USB 3.0으로 옮기면 처리량과 전류 소모가 함께 늘어납니다.

## 9. 중단된 세션 복구

어떤 파일도 덮어쓰지 않습니다. 새 세션은 항상 새 디렉터리를 받고, 비정상 종료된 데몬은 예전 디렉터리에 이어 쓰지 않습니다.

```bash
find <ssd>/OASIS_DAQ/sessions -name INCOMPLETE              # 중단된 세션 찾기
oasis-daqctl summarize <session-dir> --rebuild              # manifest.json 재생성
ffprobe -count_frames <session-dir>/camera/driver_000.mkv   # 조각별 확인
```

완료된 Matroska 조각은 강제 종료에도 살아남습니다. 진행 중이던 조각도 대개 재생 가능하지만 잘려 있습니다(시험에서 약 1500프레임 중 821프레임 복구).

## 10. 제거

```bash
sudo bash "1_Sensor_Acquisition/raspberry_pi/scripts/uninstall.sh"    # 유닛과 udev 규칙 제거
```

SSD에 기록된 세션은 건드리지 않습니다.
