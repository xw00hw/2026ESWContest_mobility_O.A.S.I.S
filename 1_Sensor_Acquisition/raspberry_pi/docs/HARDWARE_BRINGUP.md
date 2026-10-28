[← Raspberry Pi 센서 수집기](../README.md)

# 하드웨어 구성과 측정

> 수집기를 실제 벤치 하드웨어에 올리며 확인한 장치 식별, 프로토콜, 성능 측정값과 그 명령을 정리합니다.
> 이 문서의 모든 수치는 실측값이며, 데이터시트에서 옮긴 값은 없습니다.

---

## 1. 호스트

| 항목 | 값 |
|---|---|
| 보드 | Raspberry Pi 5 Model B Rev 1.1 |
| OS | Ubuntu 24.04.4 LTS (noble), `aarch64` |
| 커널 | 6.8.0-1047-raspi |
| Python | 시스템 3.12.3, **수집기 실행용 가상환경 3.11.16** |
| 코어 / RAM | 4 / 7933 MB |
| 유휴 SoC 온도 | 41–47 °C, `vcgencmd get_throttled=0x0` (시험 내내 스로틀링 없음) |

## 2. 연결 장치 식별

```text
$ lsusb
1546:01a7 U-Blox AG [u-blox 7]                    -> GPS
1a86:7523 QinHeng Electronics CH340 serial conv.  -> IMU 브리지
0c45:6366 Microdia Webcam Vitade AF               -> Arducam OV9281 카메라
04e8:4001 Samsung Electronics PSSD T7             -> 외장 SSD
```

| 역할 | 커널 노드 | 고정 식별자 | udev 심볼릭 링크 |
|---|---|---|---|
| GPS | `/dev/ttyACM0` | `usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00` | `/dev/oasis_gps` |
| IMU | `/dev/ttyUSB0` | `usb-1a86_USB_Serial-if00-port0` | `/dev/oasis_imu` |
| 카메라 | `/dev/video0` | V4L2 카드 `Arducam OV9281 USB Camera` | `/dev/oasis_camera` |

- **두 시리얼 어댑터 모두 USB 시리얼 번호를 보고하지 않으므로** udev 규칙은 VID/PID로 매칭합니다.
- `1a86:7523`은 *범용* CH340 ID라 CH340 장치가 하나 더 연결되면 충돌합니다.
  이 경우를 위해 `udev/99-oasis-daq.rules`에 물리 포트 고정(`ENV{ID_PATH}`) 규칙을 주석으로 넣어 두었습니다.
- `/dev/video1`도 존재하지만 캡처 포맷이 없는 UVC 메타데이터 노드입니다. 카메라 udev 규칙은 `ATTR{index}=="0"`으로 매칭하고
  장치 탐색에서도 걸러내므로 실제 캡처 노드만 열립니다.

## 3. GPS — u-blox 7

수신기의 시작 배너로 식별했습니다.

```text
$GPTXT,01,01,02,u-blox ag - www.u-blox.com*50
$GPTXT,01,01,02,HW  UBX-G70xx   00070000 FF7FFFFFo*69
$GPTXT,01,01,02,ROM CORE 1.00 (59842) Jun 27 2012 17:43:52*59
$GPTXT,01,01,02,PROTVER 14.00*1E
$GPTXT,01,01,02,ANTSTATUS=OK*3B
```

| 항목 | 내용 |
|---|---|
| 인터페이스 | **USB CDC-ACM 네이티브**. 호스트 측 baud는 무관하며, 같은 수신기의 USB-UART 변형도 동작하도록 `gps.baud: 9600`을 유지 |
| 출력 문장 | `GGA, RMC, VTG, GSA, GSV, GLL, TXT` |
| 주기 | 위치 **1 Hz** → 수집기는 **초당 2행** 기록 (GGA 1행 + RMC 1행, 각각 최신 보완 필드를 이어받음) |
| 체크섬 | 모든 시험에서 **실패 0건** (내구성 시험 한 번에만 792문장) |
| 실내 | **fix 없음**. `GPGGA` quality `0`, `GPRMC` status `V`, 위성 3–4개 추적, SNR 22–23. [LIMITATIONS](LIMITATIONS.md) "GPS 측위" 항목 참고 |

v0.1은 수신기 설정을 변경하지 않습니다.

## 4. IMU — WITMOTION WIT 표준 프로토콜 (CH340 경유)

프로토콜은 "바이트가 들어왔다"가 아니라 **프레임 품질**로 확인합니다.
`oasis-daqctl check --probe --probe-imu-baud`가 후보 baud를 차례로 시험하며, 결과는 명확합니다.

| baud | 바이트 | 유효 프레임 | 체크섬 오류 | 버린 바이트 | 프레임 비율 | 판정 |
|---|---|---|---|---|---|---|
| **9600** | 1220 | **110** | 0 | 0 | **0.992** | **WIT 확인** |
| 115200 | 3072 | 0 | 0 | 3062 | 0.000 | 기각 |
| 57600 | 3072 | 0 | 0 | 3062 | 0.000 | 기각 |
| 38400 | 2304 | 0 | 0 | 2294 | 0.000 | 기각 |
| 19200 | 1670 | 0 | 0 | 1660 | 0.000 | 기각 |

틀린 baud에서도 바이트는 들어오지만 유효 프레임은 하나도 나오지 않습니다.
그래서 탐색은 "무언가 도착했는가"가 아니라 `frames_ok × 11 / bytes_received`로 점수를 매깁니다.

### 프레임 형식

11바이트, 페이로드는 little-endian, 체크섬은 앞 10바이트 합의 하위 바이트입니다.

```text
0x55 <type> d0 d1 d2 d3 d4 d5 d6 d7 <checksum>
관측된 type: 0x51 acc, 0x52 gyro, 0x53 angle, 0x54 mag
```

실제 프레임 `55 51 59 03 8e fb 49 fa 23 0a fb` 해석 예:

```text
sum(first 10) & 0xFF = 0xFB  == declared 0xFB          checksum OK
ax = 0x0359 =  857 -> 857/32768*16 g   =  +0.4175 g
ay = 0xfb8e = -1138 -> -0.5557 g
az = 0xfa49 = -1463 -> -0.7153 g
|a| = 0.9971 g                                          정지 상태에서 물리적으로 타당
temp = 0x0a23 = 2595 -> 25.95 °C
```

| 항목 | 내용 |
|---|---|
| 속도 | **9600 baud, 패킷 종류별 10 Hz** (전체 40.7 프레임/s) |
| 체크섬 | 모든 시험에서 **실패 0건** (83 s 세션 한 번에 3234 프레임) |
| 참고 자료 | WITMOTION 공식 프로토콜 문서(`WitStandardProtocol_JY901`)를 문서로만 참고. 프로토콜 처리에 서드파티 pip 패키지는 쓰지 않음 |
| 한계 | 이 baud에서 10 Hz가 실제 상한. [LIMITATIONS](LIMITATIONS.md) "IMU 출력 10 Hz" 항목 참고 |

원시 시리얼 바이트는 파싱 전에 `imu/raw_*.bin`에 보관합니다. 프로토콜 해석이 틀렸더라도 원본 스트림으로 언제든 다시 재생할 수 있습니다.

## 5. 카메라 — Arducam OV9281 USB (글로벌 셔터, 흑백)

```text
$ v4l2-ctl --device=/dev/video0 --list-formats-ext
[0]: 'MJPG'  1280x800 @120/100 fps   1280x720 @120   800x600 @120/100
             640x480 @120/100        320x240 @120/100
[1]: 'YUYV'  1280x800 @10 fps        1280x720 @10 fps
```

| 항목 | 내용 |
|---|---|
| YUYV | 작업 해상도에서 10 fps — **사용 불가**. MJPG만 실용적 |
| 프레임률 | 1280x800에서 **100, 120 fps만 제공**. 30 fps를 요청해도 100이 됨 (`v4l2-ctl --set-parm=30` → `Frame rate set to 100.000 fps`). 저속 모드가 없으므로 100 fps로 캡처하고 **기록 시 솎아냄** |
| 실측 캡처 | ffmpeg 직통 **100.2 fps** (10 s에 1002프레임), 동작 중인 수집기 **약 101 fps**, OpenCV 원시 경로 92 fps |
| 데이터량 | JPEG 평균 약 37 kB, 원시 캡처 약 4.4 MB/s (≈16 GB/h) |
| 기본 기록 | `record_decimation: 4` → **25 fps 기록, ≈3.9 GB/h**. 모든 프레임을 남기려면 1로 설정 |

**채택한 프로파일: 1280x800 MJPG, 100 fps 캡처, 4프레임마다 1개 기록, 60 s Matroska 조각.**

Matroska는 MP4 대신 시험으로 검증해 선택했습니다. 데몬을 `SIGKILL`로 강제 종료한 뒤에도
**진행 중이던 조각을 포함한** 세 조각 모두 재생 가능했고, `ffprobe`는 미완성 조각에서 821프레임을 세었습니다.

## 6. 외장 SSD — Samsung PSSD T7

```text
/dev/sda1  exfat  LABEL="T7"  931.5 G
udisks2가 /media/<user>/T7 에 마운트
  rw,nosuid,nodev,relatime,uid=1000,gid=1000,fmask=0022,dmask=0022,errors=remount-ro
```

- 수집기는 이미 마운트된 exFAT 볼륨(라벨 `T7`)의 마운트 지점을 그대로 사용하고, 그 안에 `OASIS_DAQ/`만 만듭니다.
  포맷·재파티션·삭제는 하지 않으며 볼륨의 기존 파일은 건드리지 않습니다.
- 측정 시점 여유 공간: **363 GB** (569 GB 사용 중).
- 이 볼륨의 fsync 비용: 64 KiB에 **27 ms** (microSD 루트는 6.9 ms). 15 s 간격 fsync라면 코어 1개의 1 % 미만입니다.
- **부팅 시가 아니라 데스크톱 로그인 후에 마운트됩니다.** [LIMITATIONS](LIMITATIONS.md) "외장 SSD 마운트 시점" 항목 참고.
- 시험 중 갑작스러운 분리가 일어나면 커널 로그에 다음이 남습니다. 볼륨과 기존 데이터는 이후에도 온전했고, 진행 중이던 세션만 잃었습니다.
  ```text
  exFAT-fs (sda1): Volume was not properly unmounted
  sd 0:0:0:0: [sda] Synchronize Cache(10) failed
  ```

## 7. 가속기

수집기 호스트에는 `hailortcli`가 없고 `lspci`에도 Hailo 장치가 보이지 않습니다. Hailo 관련 항목은 설치·제거·재설정하지 않았습니다.
MediaPipe는 TensorFlow Lite XNNPACK 델리게이트로 CPU에서 동작하며 v0.1 목표를 만족하므로, Hailo는 의존성이 아닙니다.

## 8. 소프트웨어 의존성

| 패키지 | 버전 | 용도 |
|---|---|---|
| `openssh-server` | 1:9.6p1-3ubuntu13.18 | 노트북에서 원격 접속 |
| `git` | 2.43.0 | 소스 관리, 세션 `metadata.json`의 commit 기록 |
| `ffmpeg` | 7:6.1.1 | 녹화 경로의 조각(segment) 먹서 |
| `v4l-utils` | 시스템 패키지 | 카메라 기능 조회 |
| `mediapipe` | **1.0.1** | 자세 랜드마크 추정 (py3.11용 aarch64 휠 존재) |
| `opencv-contrib-python` | 5.0.0.93 | mediapipe 의존성 |
| `PyYAML` | 6.0.3 | 설정 파일 |

**MediaPipe 1.x에는 예전 `mp.solutions` API가 없습니다.** Tasks API만 있으며 `.task` 번들이 필요합니다.
번들은 `scripts/fetch_pose_model.sh`가 `~/.local/share/oasis-daq/models/`로 받으며, git에는 넣지 않습니다.

### 자세 추정 성능 (`pose_landmarker_lite`, Pi 5)

| 입력 | 지속 처리율 | 추론 p50 | 추론 p95 |
|---|---|---|---|
| 640x400 | 13.3 fps | 66 ms | 94 ms |
| 256x256 | 12.4 fps | 69 ms | 101 ms |

MediaPipe가 내부에서 크기를 다시 조정하므로 입력 해상도의 영향은 작습니다. 랜드마크 정밀도를 위해 640x400을 씁니다.
`pose.target_fps`는 여유를 두어 **10**으로 설정되어 있으며, 전체 세션 부하에서 자세 작업자는 9.3–9.7 fps, p95 ≈ 70 ms를 유지합니다.

## 9. 구성 명령

저장소 루트에서 실행합니다.

```bash
bash "1_Sensor_Acquisition/raspberry_pi/scripts/inspect_hardware.sh"            # 읽기 전용 전체 점검
bash "1_Sensor_Acquisition/raspberry_pi/scripts/fetch_pose_model.sh" lite       # 약 6 MB 모델
sudo bash "1_Sensor_Acquisition/raspberry_pi/scripts/install.sh"                # udev + systemd
oasis-daqctl check --probe --probe-imu-baud                                     # 전체 확인
```

벤치에서 마지막 명령의 정상 결과는 `0 problem(s), 1 warning(s)`입니다. 경고 1건은 실내 GPS fix 없음입니다.
