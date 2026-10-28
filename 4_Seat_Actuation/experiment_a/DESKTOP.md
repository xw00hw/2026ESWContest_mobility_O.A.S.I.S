[← 보드 통신 · 차량 입력 검증 · 백업 제어기](README.md)

# 데스크톱 앱 (BeamNG Bolster A)

> PC에서 백업 제어기를 단독 실행하는 데스크톱 앱입니다. 주행 시뮬레이터(BeamNG) 데이터와 보드 USB를 기다렸다가 자동으로 볼스터 제어를 시작하고, 차량·압력·제어 기록을 함께 남깁니다.
> 충격 의심 시에는 모터만 끄고 기록은 계속하며, 기록은 사용자가 `저장`이나 `버리기`를 고를 때만 확정합니다.

---

## 1. 동작 개요

바탕화면의 `BeamNG Bolster A.lnk`를 실행하면 다음 순서로 진행합니다.

```text
실행 ─► 미저장 기록 확인 ─► 차량 수집기 소켓·원본 파일 준비 ─► 보드 USB 대기
     ─► 통합 펌웨어·PWM OFF 확인 ─► 유효 차량 입력 2초 연속 ─► 한 번 자동 ARM(중립)
     ─► 백업 제어 + 압력·제어 기록 ─► 중지 ─► 저장 / 버리기
```

- 정지 차량에서는 중립을 유지하고, 주행 중 횡가속도에 따라 볼스터를 움직입니다.
- 게임 로딩 대기에는 시간 제한이 없습니다. 제어를 시작할 때 짧은 알림음을 냅니다.
- 별도의 ARM 버튼은 없습니다. 최초 장치 설정을 마친 앱을 실행하는 것이 이 한 번의 자동 시작을 허가하는 것입니다.
- 오류·통신 복구 후 자동으로 다시 ARM하거나 재시작하지 않습니다.

## 2. 최초 장치 설정

처음 한 번은 **CAD 한계를 포함한 백업 제어용 펌웨어 업로드와 장치 설정**이 필요합니다. 창의 `장치·설정 확인`에서 다음 항목을 실제로 확인해 저장하면 이후 실행부터는 바로 대기 후 시작합니다.

| 확인값 (`confirmations`) | 확인 내용 |
|---|---|
| `hardware_mapping_checked` | CAD 운동축·링크, 배선 |
| `pitch_direction_checked` | 영점과 Pitch 방향 |
| `right_bolster_fastened` | 오른쪽 볼스터 체결 |
| `neutral_start_clear` | 중립 자세에서 간섭 없음 |
| `trial_settings_reviewed` | 시험 설정 검토 |

- 프로그램은 확인값을 임의로 `true`로 바꾸거나 펌웨어를 자동 업로드하지 않습니다. 기본 후보 설정에서는 5개 모두 `false`입니다.
- 보드 통합 이미지는 `stm32/build_fallback/fallback_control_h723.elf` / `.bin`입니다. 압력 전용 이미지·측정 스테이션용 이미지와는 프로토콜이 다르므로 이 이미지로만 시작합니다.
- PWM–서보 출력축 각도는 500~2500 µs ↔ 270°로 가정합니다. 실제 서보의 범위나 혼 영점이 다르면 CAD로 계산한 구조물 각도도 달라지므로, 장치 확인 항목에 이 보정을 포함합니다. 명령 상태 표만으로는 이 보정을 확인할 수 없습니다.

## 3. 사용 순서

1. `BeamNG Bolster A`를 실행합니다. 이전 미저장 임시 기록이 있으면 복구 목록에서 먼저 `저장` 또는 `버리기`를 고릅니다. 미해결 기록이 있는 동안 자동 시작·재연결은 차단됩니다.
2. 통합 펌웨어 보드 USB가 있으면 자동 선택하고, 없으면 연결을 기다립니다. ST-LINK가 여러 개면 실행 프로필의 `port`를 지정합니다(예: `COM5`). Bluetooth나 다른 USB 직렬 장치는 임의로 선택하지 않습니다.
3. BeamNG에 차량을 불러옵니다. 기존 수집기 모드(v0.3)가 데이터를 보내면 원본 저장과 유효성 확인이 시작됩니다. 이미 설치되어 있으면 다시 설치할 필요가 없습니다.
4. 연결이 안정되면 모터 제어가 자동 시작됩니다. 압력 스캔은 보드 연결 시 시작되어 구동 전후 값을 모두 남깁니다.
5. 충격 의심 조건이 성립하면 **모터만 OFF하고 같은 세션의 차량·압력 기록은 계속**합니다. 화면은 OFF 확인 중인 `IMPACT_OFF_PENDING`과 확인된 `RECORD_ONLY`를 구분하며, 이 세션에서 다시 구동하지 않습니다.
6. `중지`를 누르면 모터 `FB_OFF`, 압력 `DISABLE`, 기록 스레드 종료를 기다립니다. 중지는 영구 저장을 뜻하지 않습니다. 기록 중에는 저장·버리기를 할 수 없습니다.
7. 종료한 임시 기록의 `저장` 또는 `버리기`를 고릅니다. 모든 미저장 기록을 처리한 뒤 `다시 연결`을 누르면 새 임시 세션으로 연결하고, 유효 차량 데이터 2초 후 한 번 자동 시작합니다.

| 상황 | 동작 |
|---|---|
| 통신·장치·기록 오류, OFF 확인 실패 | 전체 실행 종료 |
| 구동 중 200 ms 입력 공백, 잘못된 시각·스트림·파생값 | 즉시 OFF 후 종료 |
| 충격 의심 | 모터만 OFF, 기록 유지 (`RECORD_ONLY`) |
| 창 닫기 | 실행 중지, 임시 기록은 남김 (자동 저장·삭제 없음) |
| 유효한 큰 횡가속도 | 충격이 아니라 최대 목표에 포화, 기구·속도 제한 유지 |

충격 의심은 평면 합성가속도 25 m/s² 이상·급변 150 m/s³ 이상·연속 0.06초·구간 적분 1.5 m/s를 **모두** 만족할 때이며, 합성 60 m/s² 이상이면 즉시 모터 OFF합니다. 확정 충돌 판정이 아닙니다. 상세 조건은 [RUNTIME.md](RUNTIME.md)를 참고합니다.

> - 기존 `BeamNG Data Collector`와 이 앱은 같은 UDP 64256을 쓰므로 동시에 실행하지 않습니다. 앱은 단일 인스턴스로 동작하며, 수집기 소켓·파일 준비가 실패하면 모터 포트를 열지 않습니다.
> - PWM OFF는 외부 전원 차단이나 기구 위치 유지·압박 해제를 뜻하지 않습니다.

## 4. 화면 표시와 STM 튜닝

창에는 좌·우 볼스터의 최근 명령 Yaw/Pitch와 목표 Yaw/Pitch, UR/UL/DR/DL PWM, 마지막 응답 이후 경과 시간이 표시됩니다. 이 값은 STM이 보고한 **명령 상태**이며 실제 출력축 각도·전류·토크·접촉 하중이 아닙니다. 연결 전에는 0° 대신 `—`로 표시합니다.

`STM Live Expressions 적용값` 표는 아래 6개 값과 STM 적용 revision을 읽기 전용으로 보여 줍니다. 값 수정은 STM32CubeIDE에서 `g_experiment_a_tune`으로 합니다.

| 항목 | STM 변수 (`values.*`) | 기본값 | 의미 |
|---|---|---:|---|
| Yaw 상한 | `yaw_max_mdeg` | 15° | 볼스터 구조물의 최대 닫힘 각도 |
| Pitch 상한 | `pitch_max_mdeg` | ±6° | Yaw별 CAD 허용치와 함께 적용되는 추가 상한 |
| Yaw 속도 | `yaw_rate_mdeg_s` | 2 °/s | 호스트 목표 변화율. PWM 변화율 제한은 별도로 유지 |
| Pitch 속도 | `pitch_rate_mdeg_s` | 2 °/s | 〃 |
| 최대 반응 횡가속도 | `full_scale_ay_mmps2` | 2.942 m/s² | 이 횡가속도에서 설정한 최대 반응에 도달 |
| Pitch 응답 배율 | `pitch_scale_permille` | 1.0 | 가속도 등가 기울기 신호에 곱하는 배율 (체형 보정값 아님) |

**변경 절차**

1. 앱에서 `중지`해 STM 출력 OFF를 확인하고, 기록의 `저장` 또는 `버리기`를 고릅니다.
2. CubeIDE에서 실행을 유지한 채 6개 값을 수정하고, **마지막에** `request_seq`를 증가시킵니다.
3. `applied_seq`와 결과를 확인한 뒤 앱의 `다시 연결` 또는 재실행으로 새 세션을 시작합니다.

구동 중 변경은 거절되며, 호스트는 연결 때 읽은 STM 설정을 세션 동안 고정합니다. 전체 유효 설정과 revision은 manifest에 기록합니다. 변수 범위와 상세 절차는 [Live Expressions 안내](../stm32/EXPERIMENT_A_LIVE_TUNING.md)를 따릅니다.

장치 없이 화면만 확인하려면 다음을 실행합니다. `--preview`는 읽기 전용 화면, `--check-ui`는 숨긴 창 생성 검사이며 둘 다 UART·UDP·수집 스레드를 열거나 설정을 바꾸지 않습니다.

```powershell
python "$env:LOCALAPPDATA\OASIS\ExperimentA\bootstrap.py" --profile "$env:LOCALAPPDATA\OASIS\ExperimentA\desktop_profile.json" --preview
```

## 5. 기록 저장 위치

| 단계 | 위치 |
|---|---|
| 수집 중·미저장 | `%LOCALAPPDATA%/OASIS/ExperimentA/pending_sessions/V<번호>/A_<timestamp>_<id>/` |
| `저장` 선택 후 | 바탕화면 `BeamNG Bolster A Data/V<번호>/A_<timestamp>_<id>/` |

- 임시 기록은 실제 디스크에 쓰므로 창을 닫거나 프로세스가 비정상 종료돼도 복구할 수 있습니다. 중지나 오류만으로 최종 폴더를 만들지 않습니다.
- `버리기`는 선택한 임시 기록만 삭제합니다. 같은 초에 여러 번 실행해도 고유 ID가 달라 덮어쓰지 않습니다.
- `기록 폴더 열기`는 현재 임시 또는 저장된 기록 위치를 엽니다.

```text
BeamNG Bolster A Data/
├── V1/
│   └── A_<timestamp>_<id>/
└── V2/                                   # 코드 업데이트 후 별도 버전
    └── A_<timestamp>_<id>/
        ├── recording.json                # 임시/저장 상태·원래 경로·소유 기록 ID
        ├── recording_relocation.json     # 저장 시 생성: 임시 경로 → 저장 경로
        ├── experiment.json               # V번호·코드 해시·설정·라벨·시작/종료·수집기 경로
        ├── beamng/sessions/<date>/session_<id>/
        │   ├── raw/telemetry.jsonl       # 수집기 원본 패킷
        │   ├── vehicle/vehicle.csv       # 속도·종/횡가속도·조향·페달 등
        │   ├── imu/imu.csv, obd/obd.csv  # 수집기의 파생 기록
        │   └── metadata.json, session_summary.json
        └── control/
            ├── control.jsonl             # 입력 프레임·제어 진단·목표·보드 명령 상태
            ├── pressure.jsonl            # 압력 원문·파싱·전체 스캔·품질 이벤트
            ├── pressure_raw.csv          # 채널 그룹별 raw ADC·status·시각
            ├── pressure_sweeps.csv       # 완성된 12채널 스캔 + 채널별 시각
            ├── manifest.json             # 설정·해시·OFF 확인·수집 통계
            └── INCOMPLETE                # 정상 종료 시 제거, 오류·미완료 시 유지
```

저장할 때 원본 CSV/JSONL과 manifest의 바이트는 수정하지 않습니다. 그래서 내부에 기록된 절대 경로는 임시 폴더를 가리킬 수 있으며, `recording_relocation.json`과 `experiment_a.recording_lifecycle.resolve_recording_path()`로 현재 기록 폴더 안의 경로로 해석합니다. `experiment.json`의 `recording_retention_at_capture=pending`은 수집 시점 상태이고, 현재 보존 상태는 `recording.json`이 기준입니다. `저장`은 오류 기록을 정상 데이터로 바꾸지 않으며 `INCOMPLETE`도 그대로 둡니다.

## 6. 압력 기록과 시간축

| 항목 | 내용 |
|---|---|
| 샘플링 | 명목 채널당 100 Hz. 세 뱅크의 같은 채널을 한 행으로 보내므로 원시 행 약 400행/초, 완성 스캔 약 100개/초 |
| 값 | 원시 ADC·상태 코드 보존, 명목 전압 = `ADC × 3.3 / 4095`. 힘·Pa 교정값 아님 |
| 채널 | A0 = 좌판, A1 = 측면(볼스터), A2 = 등판. A1의 C0/C1/C2/C3 = DR/UR/DL/UL |
| 시간축 | 차량·압력·제어의 `host_monotonic_ns`는 같은 PC의 `perf_counter_ns` |

- 차량의 원래 수신 시각·simulation time·stream/vehicle/sequence ID와 압력 MCU의 `t_ms`·sequence를 따로 보존합니다. 12채널은 동시에 샘플링되지 않고, 압력 호스트 시각에는 UART 전송 지연이 포함되므로 서로 다른 시계를 숫자만 보고 직접 결합하지 않습니다.
- 압력 원문·품질·드롭 수를 남겨 누락을 정상 측정처럼 숨기지 않습니다.
- 압력값·기준선·접촉 여부는 제어기 입력이 아닙니다. 압력이 달라도 같은 차량 입력에는 같은 모터 명령이 나옵니다.
- 모터와 압력은 UART 소유자 하나가 처리하지만 `FB_PING`과 압력 `PING`의 lease는 독립입니다. 명시적 보드 고장·통신 오류는 종료하고, 압력 자료 품질 문제는 기록만 합니다. 로그 큐 초과·디스크 오류는 실행을 종료하고 미완료 표시를 남깁니다.

## 7. 설정과 설치

| 파일 | 위치 |
|---|---|
| 실행 프로필 | `%LOCALAPPDATA%\OASIS\ExperimentA\desktop_profile.json` |
| 시험 설정 | `%LOCALAPPDATA%\OASIS\ExperimentA\trial_settings.json` |
| 후보 설정 원본 | `experiment_a/config.candidate.json` |

- 현재 제어는 착석자의 키·체중에 따른 보정을 사용하지 않습니다. 실제 실행한 설정은 각 기록의 manifest에 복사합니다.
- 실행 프로필의 `labels`에 참가자·시나리오·반복 이름을 둘 수 있습니다. 기록 구분용이며 제어 입력이 아닙니다. 수집기의 차량·시나리오 metadata도 함께 보존됩니다.
- 제어 방식 비교 시에는 같은 시나리오·참가자·기구·각도/속도·평가 구간을 쓰고 실제 압력·자세 지표로 비교합니다. 제어기의 가속도 등가 기울기는 실측 자세가 아닙니다.

재설치는 `4_Seat_Actuation` 폴더에서 실행합니다. 기존 개인 설정은 보존합니다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File experiment_a/Install_Desktop.ps1 -PythonPath <venv>/Scripts/python.exe -CollectorRoot <BeamNG_Collector_v0_3>
python "$env:LOCALAPPDATA\OASIS\ExperimentA\bootstrap.py" --profile "$env:LOCALAPPDATA\OASIS\ExperimentA\desktop_profile.json"
```

설정을 기준값으로 되돌릴 때만 `Install_Desktop.ps1 -ResetSettings`를 **명시적으로** 사용합니다. 기존 설정과 프로필을 `*.before_reset_<timestamp>_<id>.json`으로 먼저 보존한 뒤, 새 후보 설정과 미확인 장착값으로 바꿉니다. 개인 체격 라벨은 `unpersonalized`로 바뀌고 원래 라벨은 백업과 기존 기록에 남습니다. 새 릴리스 게시에 실패하면 기존 시험 설정을 복원합니다. 일반 업데이트에서는 이 옵션을 쓰지 않으며, 후보 JSON을 수정해도 설치된 시험 설정을 자동으로 덮어쓰지 않습니다.

## 8. 버전 관리 (V1 → V2 → …)

바로가기는 고정 실행기 `bootstrap.py`를 열고, 실행기는 선택된 버전의 코드 해시를 검증한 뒤 그 사본을 실행합니다. 프로젝트 소스를 수정해도 이미 설치된 사본은 변하지 않습니다.

1. 코드를 수정·검증합니다. 개발 원본 경로는 실행 프로필의 `source_root`입니다.
2. `중지`와 기록의 `저장`/`버리기`를 마친 뒤 **업데이트**를 누릅니다. 최초 장치 확인 대기 중에도 가능하며, 구동 중에는 버튼이 비활성화됩니다.
3. 실행 코드가 달라졌으면 새 번호(V2, V3 …)로 사본을 만들고, 같으면 현재 버전을 재사용합니다. 복사·Python 구문·해시 검증에 성공한 뒤에만 활성 버전을 바꿉니다.
4. 창을 닫고 같은 바로가기를 다시 실행하면 새 버전 이름이 창에 표시되고, 저장한 기록은 해당 버전 폴더에 보존됩니다. 실행 도중 코드를 교체하거나 모터를 자동 재시작하지 않습니다.

| 파일 | 내용 |
|---|---|
| `releases/V<번호>/release.json` | 실행 모듈·공유 의존성·기본 설정의 파일별 해시와 전체 해시 |
| `current.json` | 다음 실행 버전 선택 |

두 경로 모두 설치 폴더 `%LOCALAPPDATA%/OASIS/ExperimentA/` 기준입니다.

- 이 기능은 PC의 수정된 프로젝트 코드를 설치하며, 인터넷·Git 원격 변경을 내려받지 않습니다. 다른 앱 창이 실행 중이면 외부 설치는 잠금으로 거절합니다.
- 문서·테스트·캐시만 바뀌면 번호를 올리지 않습니다. 설정만 수동 조정하면 번호는 그대로이며 기록된 설정으로 구분합니다.
- 시험 설정과 장착 확인값은 업데이트 때 그대로 유지합니다. 실행마다 사용한 설정 전체와 코드 버전·해시를 `experiment.json`과 `control/manifest.json`에 남깁니다.
- 버전 관리 이전의 기존 데이터는 원래 위치에 두며 V1 기록으로 옮기지 않습니다.
- 저장소 코드를 `python -m experiment_a.desktop`으로 직접 실행하면 `DEV`로 표시·분리되므로, 일반 실험에는 설치된 바로가기를 사용합니다.

검증 결과는 [VALIDATION.md](VALIDATION.md)를 참고합니다.
