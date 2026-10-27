# 2026-10-28 착석 시운전 재현 코드

오늘 시험에 사용한 독립 실행 래퍼, RTMPose 네이티브 표시창, 카메라 중립 수집기, PC 그래프 실행기를 보존한다. 이 디렉터리를 체크아웃하거나 테스트하는 것만으로 장치가 동작하지 않는다. 실행에 필요한 가중치, 카메라 중립 파일, SSH 키, 개인별 원시 압력과 영상은 포함하지 않는다.

## 확인한 제어 조건

| 항목 | 값 / 의미 |
|---|---|
| 중립 | UR/D3=1800, UL/D4=1800, DR/D5=1200, DL/D6=1200 µs |
| 서보 / 기구 | DS5180 270°, R8 역기구학으로 패드 각도에서 PWM 변환 |
| 시험 상한 | Yaw 0~20°, Pitch 0°, STM Yaw 속도 설정 10°/초 |
| 시간 | ARM부터 최대 300초, 중립 안정화 2초와 기준선 2초 포함 |
| G 기본곡선 | 측정 0.01g=0°, 0.0625g=10°, 1g=20°; 구간 선형 |
| 예측 / RL | 예측만일 때 최대 10°·100ms 확인; actor-학습 prior 차를 최대 ±1.5° 보정, 1g에서 보정 0° |
| 필터 | signed Yaw에 tau=0.3초 필터 한 번; 속도 1m/s 미만 0° |
| 압력 | 실제 원시 입력 유지; 각 A1 패드 2500 count 이상이면 해당 쪽 열림 상태를 래치 |
| 기준선 | 이 시험만 40 count 표준편차 초과를 경고로 기록; 신선도·표본수·시간범위 검사는 유지 |
| 카메라 | 선택 입력, 양어깨·골반 및 명시적 중립·좌우 부호 보정 필요; 미입력/오래된 입력은 unavailable |

1g에서 20°는 **필터 전 요청각**이다. 속도 제한·통신·하위 제어가 있어 즉시 실제 축이 20°가 됐다는 뜻은 아니다. 보드가 보고하는 각도는 명령 상태이며 축 센서의 실측값이 아니다. count를 보정된 접촉 힘으로 해석하지 않는다. 카메라 추론은 Hailo-10H의 Mul_204 앞부분과 CPU의 마지막 합성곱/SimCC head를 결합한 부분 가속이다.

## 구성과 기본 설정

- `occupied_trial.py`, `raw_count_guard.py`, `startup_alignment.py`: 실제 사용했던 원본과 바이트 단위로 같다. `source_provenance.json`에 해시를 보존한다.
- `candidate.yaml`: 로컬 경로·장치 ID는 `REPLACE_WITH_...` 자리표시자이며 실행 확인은 모두 false다. MOZA 보정 확인도 false다. 이 파일 그대로는 ARM하지 않는다.
- `prepare_candidate.py`: 기존 입력 설정을 복사하여 실행 확인을 제거한다. 이 저장본은 로그 경로를 입력에서 유지하며 미지정이면 자리표시자를 넣는다. 실제 실행 확인을 자동으로 true로 바꾸는 도구는 제공하지 않는다.
- 일반 `runtime/config.yaml`의 hybrid 기본 비활성·기준선 기본 reject는 그대로다. 오늘 사용한 G 혼합 제어와 raw2500 래치는 **이 래퍼와 명시적 시험 설정**으로 실행해야 한다. 일반 app만 실행하는 것은 같은 시험이 아니다.
- 보드 리셋 후 RAM 튜닝은 초기값으로 돌아갈 수 있다. 래퍼는 정확한 영점·rev2·Yaw20·10°/초를 확인하고 다르면 ARM을 거부한다. 펌웨어 업로드/튜닝 명령은 이 패키지가 수행하지 않는다.

## 설정 검토와 실행 인터페이스

Linux Pi에서 저장소 경로를 `ESW_REPO`, 시험 Python 환경을 `PYTHON`으로 지정한다. 패키지 경로는 `PKG="$ESW_REPO/3_RL_Control/deploy/experiments/seated_20261028"`이다. 아래 첫 명령은 설정 생성만 하고 장치를 열지 않는다.

```bash
"$PYTHON" "$PKG/prepare_candidate.py" --source "$PKG/candidate.yaml" \
  --output /absolute/new_candidate.yaml --expected-tuning-revision 2
```

생성한 파일의 모델·패드 보정·STM 포트·허용 PC IP·로그 경로를 실제 설치와 맞춘다. MOZA는 현재 차량과 450° 설정을 검증한 뒤 보정 확인을 갱신한다. 카메라는 없으면 `pose.source: none`으로 바꿀 수 있다. 선택 카메라에 자동 중립은 없다. 실행 설정의 `confirmations` 및 `deployment.approved_trial`, `occupied_trial_accepted`, `operator_stop_ready`는 당시 장치 상태를 확인한 별도 실행 파일에만 기록한다. `APPROVAL` 문자열은 래퍼 상수로 정의되어 있으며, 문구만 복사해 준비 상태를 대신하지 않는다.

모든 준비가 확인된 실행 파일에 대한 **실제 모터 실행 명령 형식**은 다음과 같다. 고유한 STOP 파일을 지정하며 재시작 시 기존 파일을 지워서 재사용하지 않는다. 래퍼 잠금 디렉터리 `~/ESW_RL_FB`는 미리 존재해야 한다.

```bash
"$PYTHON" "$PKG/occupied_trial.py" \
  --runtime-root "$ESW_REPO/3_RL_Control/deploy/runtime" \
  --config /absolute/reviewed_ready_config.yaml --stop-file /absolute/unique_trial.STOP
# 중지: 위에서 지정한 STOP 파일을 만든다. 자동 재ARM은 없다.
```

`--monitor-link`는 원래 시험의 `/home/esw/ESW_RL_FB/reconnect_20261028/live_control.jsonl`만 허용한다. 다른 설치에서는 이 옵션을 생략하고 PC 모니터가 실제 생성된 세션의 `control.jsonl`을 읽도록 설정한다. 이 제약을 범용 링크 변경으로 완화하지 않았다.

## 카메라와 표시창

기존 `oasis_rt.pose_camera` 한 프로세스가 RTMPose와 localhost:8081 미리보기를 제공한다. Linux 데스크톱에서 GTK3/PyGObject/Cairo가 있는 Python으로 다음을 실행한다. 카메라를 추가로 열지 않고 기존 주석 JPEG를 읽는다.

```bash
python3 "$PKG/native/rtmpose_viewer.py" --url http://127.0.0.1:8081
```

프레임이 끊기면 마지막 정상 영상을 원래 밝기로 유지하고 WAITING·경과 시간을 표시한다. Q/Esc는 창만 닫는다. 화면에 과거 영상이 남는 것과 제어 입력의 유효성은 별개다.

`camera/collect_pose_reference.py --help`는 같은 Pi의 이미 실행 중인 producer에서만 관측을 수집한다. `--operator-confirmed`, 정확한 `--camera-id`·`--model-sha256`, 새 `--output`을 요구한다. 최소 20개 신선한 관측과 안정도를 검사하며, 기본 5초 수집이다. `--neutral FILE --side left`는 기존 중립을 수정하지 않고 부호 확인 기록만 만든다. 실제 USB 재연결은 중립을 무효화하므로 재확인이 필요하다.

PC에서는 `pc/config.example.json`을 별도 로컬 파일에 복사하여 기존 검증된 SSH 키/known_hosts와 실제 모니터 소스를 지정한다. 기본 명령은 오프라인 설정 검사만 한다. 네트워크 읽기를 시작할 때만 `--start`를 추가한다.

```powershell
python pc/start_pc_displays.py --config C:/absolute/pc_display_config.json
python pc/start_pc_displays.py --config C:/absolute/pc_display_config.json --start
```

그래프 주소는 `http://127.0.0.1:8766/`다. PC 그래프는 읽기 전용이고 제어권을 갖지 않는다. SSH host key 검증을 끄지 않는다.

## 장치 없이 검사

패키지 폴더에서 `PYTHONPATH`와 `OASIS_TRIAL_TEST_RUNTIME`을 저장소 `3_RL_Control/deploy/runtime` 절대경로로 지정한 뒤 각각 실행한다. 실제 모델 검사는 외부 `OASIS_TEST_MODEL_DIR`이 없으면 명시적으로 skip한다.

```bash
python -m unittest discover -s . -p 'test_*.py'
python -m unittest discover -s native -p 'test_*.py'
python -m unittest discover -s camera -p 'test_*.py'
python -m unittest discover -s pc -p 'test_*.py'
```

시험의 세션·중단 이유·실행 시간·원본 해시 및 당시 한계는 저장소의 당일 closeout 문서와 별도 PDF/로그 산출물을 따른다. 재생/단위시험 성공은 실제 착석 효과 검증을 대신하지 않는다.
