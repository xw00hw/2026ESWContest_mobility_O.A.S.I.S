# Seat control data acquisition

NUCLEO-H723ZG 한 보드로 DS 서보 4개와 압력매트 12채널을 제어하고, Windows GUI에서
좌우 구조물 Yaw/Pitch·압력·선택 카메라 영상을 함께 기록하는 코드입니다.
이 폴더를 따로 복사해도 GUI의 파서·기구학·설정·실행 스크립트를 사용할 수 있습니다.

| 경로 | 내용 |
|---|---|
| [gui](gui/README.md) | 수동 좌우 제어, AUTO 357/784, 압력 표시, 영상/CSV 동시 기록 |
| [stm32](stm32/README.md) | C 펌웨어·핀 설정·CMake·빌드/업로드 스크립트 |
| [fallback](fallback/README.md) | BeamNG 위치 기반 폴백 + 압력 증가 감시; 영상/RL 제외, 전용 펌웨어·오프라인 검증 |
| [experiment_a](experiment_a/DESKTOP.md) | BeamNG Bolster A 바탕화면 자동 실행; BeamNG 전용 제어 + 차량·압력 관측 동시 저장, Yaw25°·가변 Pitch·4/3°/s, 최초 실장 검증 대기 |
| [gui/processing](gui/processing/README.md) | 2026-10-25 세 세션의 압력·포즈 후처리 코드와 별도 의존성 |
| [gui/tests](gui/tests) | 모의 통신·기록·자동 스캔·Python/C 기구학 시험 |
| [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json) | 원본 코드 위치·버전·패키징 변경 내역 |

원시 데이터·영상·모델 가중치·SDK·빌드 산출물은 포함하지 않습니다.
`gui/processing`은 기존 세션 ID 3개를 대상으로 한 오프라인 파이프라인이며,
새 세션 후처리에는 스크립트의 세션 목록/계획 검사를 맞춰야 합니다.

## GUI 실행

Windows에 Python 3.10 이상(Tk 포함)을 설치하고 `gui/Setup_Control_Station.cmd`를 실행한 다음
`gui/Open_Control_Station.cmd`를 실행합니다. Python 3.13에서 검증했습니다.
내장캠은 **Integrated Camera**, USB 카메라는 장치 목록에서 선택합니다.

1. 펌웨어를 올린 보드의 COM 포트에 연결합니다.
2. 카메라를 선택하고 압력·영상 기록 여부를 정합니다.
3. 수동 제어는 **구동 활성화** 후 왼쪽/오른쪽 Yaw·Pitch를 조절합니다.
4. **AUTO 357**은 선택한 쪽의 Yaw 0~20° × Pitch −8~8°를 1° 간격으로 기록합니다.
5. **양쪽 Yaw AUTO 784**는 좌우 Yaw 0~27° × 0~27°, 양쪽 Pitch 0°로 기록합니다.

AUTO는 선택한 압력·카메라를 준비하고 유효 수신을 확인한 뒤 기록·구동을 시작합니다.
기본 안정 0.5초·측정 2초이며, 완료하면 영점 복귀·HOLD 후 요약을 저장합니다.
기록은 `gui/sessions/<시각_UUID>/`에 생성됩니다. **Esc**는 현재 PWM을 유지하고 구동을 비활성화합니다.

## 배선·제어 기준

| 기능 | 보드 핀 | MCU 핀 |
|---|---|---|
| 오른쪽 상부 / 하부 서보 | D3 / D5 | PE13 / PE11 |
| 왼쪽 상부 / 하부 서보 | D4 / D6 | PE14 / PE9 |
| 공통 MUX S0/S1/S2/S3 | D8/D9/D10/D11 | PF3/PD15/PD14/PB5 |
| 공통 MUX EN | D12 | PA6 |
| MUX SIG 0/1/2 | A0/A1/A2 | PA3/PC0/PC3_C |

세 MUX에서 C0~C3를 읽어 총 12개 센서를 기록합니다. ADC는 12bit·공칭 3.3V이고 채널당 약 100Hz입니다.
서보는 모두 DS5180 270°이며 TIM1 50Hz PWM을 사용합니다.
부팅/기구 영점의 D3/D4/D5/D6는 **1800/1200/1200/1800µs**입니다.
왼쪽 `D4=θ1,L`, `D6=−θ2,L`, 오른쪽 `D3=−θ1,R`, `D5=θ2,R`입니다.
θ는 C4_R8 역기구학의 중립 대비 크랭크 각도입니다.
GUI 수동 Yaw 0~20°·Pitch ±8°, 펌웨어 Yaw 최대 27°이며 도달 불가 조합은 거부합니다.
통신 timeout 1.5초, ARM/HOLD, S커브와 PWM 변화 제한을 유지합니다.

## STM32 빌드·업로드 / VS Code

`4_Seat_Actuation.code-workspace`를 VS Code로 열면 GUI 설치/실행, STM32 빌드·장치 목록·업로드 작업을 선택할 수 있습니다.

필요 도구: STM32CubeIDE ARM GNU GCC, STM32CubeH7 **V1.13.0**, CMake ≥3.20, Ninja, STM32CubeProgrammer.
기본 도구 경로는 `stm32/CMakeLists.txt`와 `stm32/scripts/stm32.ps1`에 있습니다.
다른 PC에서는 스크립트의 `-CMakePath`, `-NinjaPath`, `-ProgrammerPath`, `-ToolchainPath`, `-CubePath`로 경로를 지정합니다.
ST HAL/CMSIS와 startup/linker 파일은 설치된 CubeH7 SDK에서 가져옵니다.

이 폴더에서 다음을 실행합니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File stm32/scripts/stm32.ps1 -Action Build
powershell.exe -NoProfile -ExecutionPolicy Bypass -File stm32/scripts/stm32.ps1 -Action List
powershell.exe -NoProfile -ExecutionPolicy Bypass -File stm32/scripts/stm32.ps1 -Action Upload -StLinkSerial '<대상 ST-LINK serial>'
```

업로드는 대상 serial을 명시해야 실행되며 빌드 후 Flash 검증·리셋까지 수행합니다.
VS Code 업로드 작업도 대상 serial을 입력받습니다.

## 기록·검증

압력·명령/상태·영상은 PC QPC(`perf_counter_ns`, ns)를 공통 시계로 저장합니다.
MCU ms, UTC wall time, 실제 캡처 시각과 CFR 표시 시각·반복 여부도 따로 남깁니다.
압력은 보정 전 ADC/추정 V, 기구 각도/PWM은 명령 상태이며 실측 위치 피드백은 없습니다.
UART는 460800 baud·8N1·LF ASCII, 영상은 30fps CFR이며 카메라가 느리면 이전 캡처를 반복합니다.
표준편차·유효 표본 수·오류·실패/미시도 상태는 자세별 요약에 남습니다.

```powershell
cmd.exe /c stm32\scripts\build_native_math.cmd
$env:OASIS_JOINT_MATH_DLL='C:\Temp\seat_control_data_acquisition\joint_math.dll'
python -m unittest discover -s gui/tests -v
# 후처리 의존성은 gui/processing/requirements.txt에 별도 정의
python -m unittest discover -s gui/processing -p test_processing.py -v
```

기존 실측과 소프트웨어 시험의 범위는 [gui/VALIDATION.md](gui/VALIDATION.md)를 참고합니다.
이 패키징 작업의 검증 결과는 [VALIDATION.md](VALIDATION.md)에 기록합니다.
