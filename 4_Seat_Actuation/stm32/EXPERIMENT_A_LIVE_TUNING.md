# 실험 A — CubeIDE Live Expressions

2026-10-27 사용자 정정 반영: DS5180 **270°** 4대 / R8 CAD 결합 한계. 현재 PC 프로그램은 펌웨어 `OASIS_FALLBACK,v2`, `servo_range_deg=270`과 동일 CAD 해시를 요구한다. 직전180° 이미지나 기존 수집용 펌웨어로는 시작하지 않는다.

## 준비된 프로젝트

- CubeIDE: `C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\stm32cubeide.exe`
- 작업공간: `C:\Users\SO\STM32CubeIDE\workspace_ExperimentA_20261027`
- 프로젝트: `ExperimentA_CAD_180` (`stm32/cubeide`). 기존 작업공간 연결을 유지하기 위한 식별자이며, 현재 소스와 ELF의 실제 사양은 **270°**다.
- ELF: `stm32/build_fallback/fallback_control_h723.elf`
- 변수 목록: `cubeide/Live_Expressions.txt` (새 작업공간에 미리 등록)
- 소스: `Core/Src/fallback_tuning.c`, `Core/Inc/fallback_tuning.h`

이 프로젝트는 소스와 미리 빌드한 ELF를 보는 프로젝트다. 자동 빌드/업로드를 실행하지 않는다. 코드 변경 후 빌드는 `scripts/stm32.ps1 -Action Build -FallbackControl`을 사용한다. 프로젝트를 여는 것만으로 장치를 연결하거나 모터에 신호를 보내지 않는다.

Git에 포함된 `cubeide/.project`, `.cproject`, `.launch`는 위 PC에서 사용한 설정의 기록이다. 다른 PC에서는 연결된 소스·ELF 경로, 도구 경로와 대상 ST-Link 설정을 다시 지정해야 한다. ELF와 IDE 작업공간 메타데이터는 Git에 포함하지 않는다. `scripts/prepare_cubeide.py`는 별도 `SMART_SEAT/MOTER/firmware/r8_debug` 템플릿을 사용하므로 ESW만 복제한 상태에서는 바로 재생성할 수 없다.

`Experiment A - attach after verified upload` 설정은 이미지 쓰기·리셋·정지·main 중단점을 끄고 Live Expressions를 켠 **첨부용** 설정이다. 실제 보드에 동일 ELF를 검증 업로드한 다음 사용한다. 다른 펌웨어에 심볼만 붙이면 변수 주소가 맞지 않으므로 사용하면 안 된다. 여러 보드가 있으면 대상 ST-Link를 명시적으로 선택해야 한다.

## 수정할 값

아래 경로 전체를 Live Expressions에 등록해 두었다. 각도는 **볼스터 구조물 Yaw/Pitch**이며 개별 모터축 각도가 아니다.

| 변수 | 초기값 | 단위 / 허용 범위 |
|---|---:|---|
| `g_experiment_a_tune.values.yaw_max_mdeg` | 15000 | 0.001°, 100~22000 |
| `g_experiment_a_tune.values.pitch_max_mdeg` | 6000 | ±0.001°, 0~6000 |
| `g_experiment_a_tune.values.yaw_rate_mdeg_s` | 2000 | 0.001°/초, 100~20000 |
| `g_experiment_a_tune.values.pitch_rate_mdeg_s` | 2000 | 0.001°/초, 100~20000 |
| `g_experiment_a_tune.values.full_scale_ay_mmps2` | 2942 | 0.001m/s², 500~8000 |
| `g_experiment_a_tune.values.pitch_scale_permille` | 1000 | 0.001배, 10~1000 |
| `g_experiment_a_tune.request_seq` | 0 | 위 값들을 쓴 **마지막에** 이전보다 큰 정수로 증가 |

예: Yaw 상한 12°는 `yaw_max_mdeg=12000`, 속도 1.5°/초는 `yaw_rate_mdeg_s=1500`이다. 설정 가능한 최대 속도와 실제 도달 속도는 다르다. 모든 출력에 별도로 2µs/20ms 제한을 적용한다.

CAD Pitch 상한은 Yaw 0°→0°, 6~15°→±6°, 20°→±2°, 22°→0°이며 중간은 선형 보간한다. 위 튜닝값으로 이 고정 한계를 넓힐 수 없다. 초기 운용은 15°/±6°다.

## 적용 순서

1. `BeamNG Bolster A`에서 **중지**하고 `g_fallback_debug.armed=0`과 출력 OFF를 확인한다. V6에서는 기록의 **저장 또는 버리기**를 별도로 선택한다.
2. CubeIDE의 MCU는 **Running** 상태로 유지하고 Live Expressions의 여섯 값을 수정한다. Pause/Step으로 값을 바꾸지 않는다.
3. `request_seq`를 마지막에 1, 2, 3…으로 증가시킨다. 실패한 번호도 소비되므로 재시도에는 새 번호가 필요하다.
4. `g_fallback_debug.tuning_result_code=1` 및 `tuning_revision=request_seq`를 확인한다.
5. 앱을 다시 실행한다. PC가 STM의 검증 적용값과 revision을 읽어 이번 실행의 고정 설정으로 사용한다. BeamNG 유효 입력 대기를 마친 뒤 동일 revision으로만 ARM할 수 있다.

구동 중 수정 요청은 BUSY로 거절하며 나중에 자동 적용하지 않는다. 중지 후 새 번호로 다시 요청해야 한다. 튜닝은 모터 명령·ARM·워치독 갱신 권한이 없다.

| `tuning_result_code` | 의미 |
|---:|---|
| 0 | 기본값 READY |
| 1 | 전체 값 검증·적용 APPLIED |
| 2 | 구동 중이라 거절 BUSY |
| 3 | 범위 위반으로 전체 거절 RANGE |
| 4 | 이전 번호 재사용/0 등 SEQUENCE 오류 |

`g_fallback_debug`는 명령 상태를 복사한 **관측용 변수**다. 수정해도 제어 입력으로 사용하지 않는다. `commanded_pose_mdeg`·`target_pose_mdeg` 순서는 LY/LP/RY/RP, `commanded_pulse_us` 순서는 UR/UL/DR/DL이다. 실제 축 위치·전류·부하 센서값이 아니다.

Live Expressions 변경값은 RAM에 있으므로 보드 재부팅/리셋 시 초기값으로 돌아간다. 실행에 사용한 값은 PC 로그에 저장된다. 기본값을 영구 변경하려면 `fallback_tuning.c`의 `DEFAULT_VALUES`를 수정하고 다시 빌드·검증 업로드한다.

## 물리 검증 상태

CAD 축·혼 길이·로드·패드 연결점은 사용자 확인으로 동일하며 모터 본체만 DS5180으로 변경됐다. 사용자 정정에 따라270° 사양을 반영했지만 500~2500µs 끝점은 실측 보정값이 아니다. 중립 펄스 1800/1200/1200/1800µs는 기존 조립 영점으로 유지했다.

STEP의 명목 간격·운동학은 검증했지만 실제 부하·온도·탄성 변형·케이스 외형·착석 압박은 미검증이다. 최초 실물 시험에서는 중립과 방향·각도 변환을 먼저 확인한다. 디버거 정지는 모터 PWM의 즉시 제거를 뜻하지 않는다.
