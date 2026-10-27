# H723 통합 펌웨어

NUCLEO-H723ZG 한 보드에서 DS 4개 PWM, 압력매트 12채널 ADC, UART 제어를 처리합니다.
초기 압력 전용 보드 펌웨어의 수집 코드를 확장해 하나의 보드로 통합했습니다.

## 압력 측정 전용 빌드

압력 센서 위치 확인용으로 모터 출력이 필요 없으면 `scripts/stm32.ps1 -Action Build -PressureOnly`를 사용합니다. CMake에서는 `-DOASIS_PRESSURE_ONLY=ON`에 해당하며, 결과는 `build_pressure_only/pressure_only_h723.elf`와 `.bin`입니다. 옵션 기본값은 OFF이므로 기존 통합 빌드는 유지됩니다.

이 빌드는 애플리케이션 시작의 첫 동작부터 D3/D4/D5/D6를 GPIO LOW로 만들고 TIM1의 PWM 출력을 끕니다. `ARM`과 `SET`은 `PRESSURE_ONLY` 오류로 거절하며 위치 제어 PWM을 출력하지 않습니다. 모터 전원 자체를 차단하는 기능은 아닙니다. 아래의 초기 위치·기구학·모터 제어 설명은 기본 통합 빌드에만 적용됩니다.

MUX D8/D9/D10/D11, EN D12, ADC A0/A1/A2와 기존 `FSR_12_SCAN,v10` 측정 프로토콜은 그대로입니다. `CONTROL_HELLO`로 전용 이미지 식별값 `OASIS_PRESSURE_ONLY,v1` 및 `motor_outputs=0,pwm=OFF`를 확인할 수 있습니다. 측정에는 `START_12,15`, `PING`, `DISABLE`을 사용합니다.

실제 빌드된 ARM ELF의 출력 차단과 명령 거절은 `scripts/test_pressure_only_elf.py`로 오프라인 검사합니다. 이 검사는 Unicorn이 필요하며, 설치 위치를 `OASIS_EMULATION_DEPS`로 지정할 수 있습니다. 물리 센서 값이나 파형의 실측을 대신하지 않습니다.

## 업로드

규칙 기반 제어용 선택 빌드는 [FALLBACK_PROTOCOL.md](FALLBACK_PROTOCOL.md)를 따릅니다. `OASIS_FALLBACK_CONTROL=ON`은 부팅 PWM OFF와 별도 `FB_*` 세션 프로토콜을 사용하며 기존 통합 v6의 자동 영점 출력과 구분됩니다. 새 호스트의 기본 실행은 dry-run이고 실제 구동 승인을 대신하지 않습니다.

이 폴더를 VS Code에서 열고 터미널에서 실행합니다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/stm32.ps1 -Action Build
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/stm32.ps1 -Action List
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/stm32.ps1 -Action Upload -StLinkSerial '<ST-LINK serial>'
```

PC의 CubeIDE ARM GCC, CubeH7 V1.13.0, Visual Studio CMake/Ninja, CubeProgrammer를 사용합니다.
경로는 `CMakeLists.txt`와 `scripts/stm32.ps1`에 있습니다.
`List`로 확인한 대상 ST-LINK를 `-StLinkSerial`로 지정해야 업로드합니다.
`Download verified successfully`가 나와야 검증 성공입니다.

## 제어

전원/리셋 후 D3/D4/D5/D6는 **1800/1200/1200/1800µs**이며 구동은 비활성입니다.
구조물 Yaw 최대 27°, Pitch ±8°를 받아 각 목표의 C4_R8 역기구학 도달 가능성을 검사합니다.
수동 GUI는 Yaw 0~20°를 사용하고 양쪽 Yaw AUTO 784는 Pitch=0°에서 0~27°를 사용합니다.
왼쪽 D4=θ1,L / D6=−θ2,L, 오른쪽 D3=−θ1,R / D5=θ2,R를 독립 계산합니다.
네 모터 모두 DS5180 270°입니다.
θ는 Viewer 로컬 크랭크 φ의 중립 대비 각도입니다.

5차 S커브의 모터 속도 목표 상한은 15°/s입니다.
별도 PWM 변화 제한은 20ms 기준 최대 2µs이며, 스케줄 지연 시 40ms까지만 반영합니다.
통신 timeout 1.5초 또는 UART/ADC 오류 시 현재 PWM을 유지하고 비활성화합니다.
다시 `ARM`을 받아야 이동합니다. 소프트웨어 정지는 외부 전원 차단이 아닙니다.

## UART

460800 baud, 8N1, LF 종결 ASCII. 하나의 RX/송신 경로를 공유합니다.

```text
CONTROL_HELLO
ARM
SET,1,5000,2000,0,0
STATE
HOLD
START_12,15
PING
DISABLE
```

`SET`은 `명령 ID,왼쪽 yaw_mdeg,pitch_mdeg,오른쪽 yaw_mdeg,pitch_mdeg`입니다. ID 범위 0~2147483647.
좌우 목표를 원자적으로 검증하며, ACK는 `#ACK,SET,id,LY,LP,RY,RP,duration_ms`, 오류는 `#ERR,id,reason`입니다.
`CONTROL_HELLO`는 `OASIS_CONTROL,v6`와 제한값·`crank_zero_us=1200,joints=2`를 반환합니다.
Yaw 27°/Pitch 8°처럼 도달하지 못하는 조합은 거부합니다. 모터별 PWM 500~2500µs와 속도 제한은 유지합니다.

```text
#STATE,mcu_ms,armed,moving,LY,LP,RY,RP,target_LY,target_LP,target_RY,target_RP,D3_us,D4_us,D5_us,D6_us,command_id,fault
```

상태는 200ms마다 전송하며 모두 **명령값**입니다. 위치 센서 피드백은 없습니다.
GUI는 250ms마다 PING을 보내고 GUI 갱신 중단 시에도 정지합니다.
압력 스캔 명령/9필드 CSV는 기존 v9와 같지만 MUX 주소선은 **D8~D11**입니다.
압력 HELLO는 v10으로 구분하여 이전 배선용 수집기가 잘못 연결되지 않게 합니다.
배선표·기록 방법은 [GUI 문서](../gui/README.md)를 따릅니다.

## 원본 기구학

사용자가 제공한 R8 Viewer의 `kinematics.js`를 Python과 C로 이식했습니다.
반경 24mm, 중심 `(−65,−36,±48)`, 부착점 `(−55,12,±48)`, `Rz(−yaw)Rx(pitch)`입니다.
원본과 SHA256은 GUI `reference/`와 `station_config.json`에 보존했습니다.
