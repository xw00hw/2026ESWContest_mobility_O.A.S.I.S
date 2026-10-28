[← 4. 시트 구동](README.md)

# 패키지 검증

> 이 폴더(`4_Seat_Actuation`)를 제출 패키지로 구성한 뒤, 펌웨어 빌드와 호스트 측 시험이 패키지 안에서 그대로 동작하는지 확인한 결과입니다.
> 패키징 검증에서는 STM32 업로드와 실제 모터 이동을 실행하지 않았습니다.

---

## 검증 결과

| 항목 | 결과 |
|---|---|
| 펌웨어 빌드 | STM32 ARM GCC + STM32CubeH7 V1.13.0 + CMake/Ninja로 `stm32/build/fsr_mux_h723.elf` 생성. 기존 newlib syscall stub 링커 경고 외 변화 없음 |
| GUI·통신·자동 기록·기구학 시험 | 복사한 `joint_math.c`로 Native C DLL을 새로 빌드한 뒤 **41개 통과** |
| 후처리 시험 | 시각 경계·crop 좌표·관측 불가 관절·feature 부호 **5개 통과** |
| 압력 파서 | 독립 압력 파서의 두 정의가 원본 `seat12_core.py`와 AST 동일. uint32 wrap·누락·중복·reset, 오류 ADC 표본의 빈 raw 필드를 별도 시험 |
| GUI 단독 실행 | 저장소 밖 임시 폴더로 복사해 import·기본 창 생성·구동 비활성 초기 상태 확인 (장치 목록은 mock, 실제 COM·카메라는 열지 않음) |
| 문서·작업공간 | 패키지 안 Markdown 링크와 VS Code workspace JSON 확인 |
| 업로드 안전 | ST-LINK serial 없이 `Upload`를 요청하면 도구·보드 접근 전에 거부 |

GUI·통신·기구학 41개 시험에는 Python과 C의 좌우 핀 매핑·기구학·속도/PWM 제한 일치, 모의 AUTO 측정(357 / 784 조합), 영상·압력 기록이 포함됩니다.

## 관련 검증 문서

| 문서 | 내용 |
|---|---|
| [`gui/VALIDATION.md`](gui/VALIDATION.md) | 측정 스테이션의 하드웨어 실측 이력 |
| [`fallback/VALIDATION.md`](fallback/VALIDATION.md) | 보드 프로토콜·브리지·펌웨어 ELF 에뮬레이션 |
| [`experiment_a/VALIDATION.md`](experiment_a/VALIDATION.md) | 백업 제어기·데스크톱 앱·CAD 한계·충격 판정 |

> 저장소에는 소스만 포함하며, 기록 세션·영상·모델·빌드 산출물은 제외합니다.
