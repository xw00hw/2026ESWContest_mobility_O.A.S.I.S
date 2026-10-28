[← STM32H723 볼스터 · 압력 통합 펌웨어](README.md)

# 실험 A 실시간 튜닝 (CubeIDE Live Expressions)

> `OASIS_FALLBACK v2` 이미지가 실행 중일 때 STM32CubeIDE Live Expressions로 **볼스터 각도·속도 상한을 재빌드 없이 바꾸는 절차**입니다.
> 튜닝값은 펌웨어가 전체를 검증한 뒤에만 적용되며, 모터 명령·ARM·watchdog 갱신 권한은 없습니다.

---

## 1. 전제 조건

| 항목 | 요구 |
|---|---|
| 펌웨어 | `OASIS_FALLBACK,v2`, `servo_range_deg=270`, 호스트와 같은 CAD 프로필 해시 |
| 서보 | DS5180 270° × 4, R8 CAD 결합 한계 |
| 사용 불가 | 180° 가정의 이전 이미지, 측정용 통합 v6 펌웨어 |

호스트 프로그램은 위 조건이 맞지 않으면 시작하지 않습니다.

## 2. CubeIDE 프로젝트

| 항목 | 값 |
|---|---|
| IDE | STM32CubeIDE 2.2.0 |
| 작업공간 | 새 작업공간 (임의 경로) |
| 프로젝트 | `ExperimentA_CAD_180` (`stm32/cubeide`). 기존 작업공간 연결을 위한 식별자일 뿐이며, 현재 소스와 ELF의 실제 사양은 **270°** |
| ELF | `stm32/build_fallback/fallback_control_h723.elf` |
| 변수 목록 | `cubeide/Live_Expressions.txt` (작업공간에 미리 등록) |
| 소스 | `Core/Src/fallback_tuning.c`, `Core/Inc/fallback_tuning.h` |

- 이 프로젝트는 소스와 미리 빌드한 ELF를 보는 용도이며 자동 빌드·업로드를 하지 않습니다. 코드를 바꾼 뒤에는 `scripts/stm32.ps1 -Action Build -FallbackControl`로 빌드합니다.
- 프로젝트를 여는 것만으로 장치에 연결하거나 모터에 신호를 보내지 않습니다.
- 저장소의 `cubeide/.project`, `.cproject`, `.launch`는 개발 PC 설정 기록입니다. 다른 PC에서는 소스·ELF 경로, 도구 경로, 대상 ST-LINK를 다시 지정해야 합니다. ELF와 IDE 작업공간 메타데이터는 저장소에 포함하지 않습니다.
- `scripts/prepare_cubeide.py`는 저장소 밖의 별도 디버그 템플릿을 사용하므로 이 저장소만으로는 프로젝트를 다시 생성할 수 없습니다.

### 디버그 설정 `Experiment A - attach after verified upload`

이미지 쓰기·리셋·정지·main 중단점을 끄고 Live Expressions를 켠 **첨부(attach) 전용** 설정입니다.

- 같은 ELF를 실제 보드에 검증 업로드한 **다음에만** 사용합니다. 다른 펌웨어에 심볼만 붙이면 변수 주소가 맞지 않습니다.
- 보드가 여러 개 연결되어 있으면 대상 ST-LINK를 명시적으로 선택합니다.

## 3. 튜닝 변수

모든 변수는 Live Expressions에 등록되어 있습니다. 각도는 **볼스터 구조물 Yaw/Pitch**이며 개별 모터축 각도가 아닙니다.

| 변수 | 초기값 | 단위 | 허용 범위 |
|---|---:|---|---|
| `g_experiment_a_tune.values.yaw_max_mdeg` | 15000 | 0.001° | 100 ~ 22000 |
| `g_experiment_a_tune.values.pitch_max_mdeg` | 6000 | ±0.001° | 0 ~ 6000 |
| `g_experiment_a_tune.values.yaw_rate_mdeg_s` | 2000 | 0.001°/s | 100 ~ 20000 |
| `g_experiment_a_tune.values.pitch_rate_mdeg_s` | 2000 | 0.001°/s | 100 ~ 20000 |
| `g_experiment_a_tune.values.full_scale_ay_mmps2` | 2942 | 0.001 m/s² | 500 ~ 8000 |
| `g_experiment_a_tune.values.pitch_scale_permille` | 1000 | 0.001배 | 10 ~ 1000 |
| `g_experiment_a_tune.request_seq` | 0 | — | 위 값을 모두 쓴 **마지막에** 이전보다 큰 정수로 증가 |

- 예: Yaw 상한 12° → `yaw_max_mdeg=12000`, 속도 1.5 °/s → `yaw_rate_mdeg_s=1500`.
- 설정한 최대 속도와 실제 도달 속도는 다릅니다. 모든 출력에는 별도로 20 ms당 2 µs 제한이 적용됩니다.
- CAD Pitch 상한은 Yaw 0° → 0°, 6~15° → ±6°, 20° → ±2°, 22° → 0°이며 사이는 선형 보간합니다. 튜닝값으로 이 고정 한계를 넓힐 수 없습니다.
- 초기 운용값은 15° / ±6°입니다.

## 4. 적용 순서

1. 데스크톱 앱(`experiment_a`)에서 **중지**하고 `g_fallback_debug.armed=0`과 출력 OFF를 확인합니다. 기록의 **저장** 또는 **버리기**를 선택합니다.
2. CubeIDE에서 MCU를 **Running** 상태로 둔 채 Live Expressions의 여섯 값을 수정합니다. Pause/Step 상태에서 값을 바꾸지 않습니다.
3. 마지막에 `request_seq`를 1, 2, 3 … 으로 증가시킵니다. 실패한 번호도 소비되므로 재시도에는 새 번호가 필요합니다.
4. `g_fallback_debug.tuning_result_code=1`, `tuning_revision=request_seq`를 확인합니다.
5. 앱을 다시 연결하거나 재실행합니다. 호스트는 STM이 검증·적용한 값과 revision을 읽어 그 세션의 고정 설정으로 쓰며, BeamNG 유효 입력 대기를 마친 뒤 **같은 revision으로만** ARM할 수 있습니다(`FB_ARM,<session_id>,<tuning_revision>`).

구동 중 수정 요청은 `BUSY`로 거절되며 나중에 자동 적용되지 않습니다. 중지 후 새 번호로 다시 요청합니다.

### 결과 코드

| `tuning_result_code` | 이름 | 의미 |
|---:|---|---|
| 0 | READY | 기본값 |
| 1 | APPLIED | 전체 값 검증 후 적용 |
| 2 | BUSY | 구동 중이라 거절 |
| 3 | RANGE | 범위 위반으로 전체 거절 |
| 4 | SEQUENCE | 이전 번호 재사용, 0 등 순번 오류 |

## 5. 관측 변수 `g_fallback_debug`

명령 상태를 복사한 **관측 전용** 변수입니다. 수정해도 제어 입력으로 쓰이지 않습니다.

| 필드 | 순서 / 의미 |
|---|---|
| `commanded_pose_mdeg`, `target_pose_mdeg` | LY / LP / RY / RP |
| `commanded_pulse_us` | UR / UL / DR / DL (D3 / D4 / D5 / D6) |
| `armed`, `fault`, `tuning_revision`, `tuning_result_code` | ARM 여부, fault 문자열, 적용 revision, 결과 코드 |

실제 축 위치·전류·부하 센서값이 아닙니다.

## 6. 값의 수명

- Live Expressions로 바꾼 값은 RAM에 있으므로 보드를 재부팅·리셋하면 초기값으로 돌아갑니다.
- 실행에 쓴 값은 PC 로그(manifest)에 저장됩니다.
- 기본값을 영구히 바꾸려면 `fallback_tuning.c`의 `DEFAULT_VALUES`를 수정하고 다시 빌드·검증 업로드합니다.

## 7. 물리 검증 상태

| 항목 | 상태 |
|---|---|
| CAD 축·혼 길이·로드·패드 연결점 | 조립 확인으로 CAD와 동일. 모터 본체만 DS5180으로 교체 |
| 270° 환산 | 반영. 단, 500~2500 µs 끝점은 실측 보정값이 아님 |
| 중립 펄스 | 현재 장착 기준 보정 영점 D3/D4/D5/D6 = 1800 / 1800 / 1200 / 1200 µs (측정용 v6는 1800 / 1200 / 1200 / 1800 µs) |
| STEP 명목 간격·운동학 | 검증 |
| 실제 부하·온도·탄성 변형·케이스 외형·착석 압박 | 미검증 |

최초 실물 시험에서는 중립 위치와 방향·각도 변환을 먼저 확인합니다. 디버거 정지가 모터 PWM의 즉시 제거를 뜻하지는 않습니다.
