# 패키지 검증 — 2026-10-25

- STM32 ARM GCC + STM32CubeH7 V1.13.0 + CMake/Ninja 빌드 성공.
  `stm32/build/fsr_mux_h723.elf`를 생성했다. 기존 newlib syscall stub linker 경고는 동일하다.
- 복사한 `joint_math.c`에서 Native C DLL을 새로 빌드하고 GUI/통신/자동 기록/기구학 시험 **41개 통과**.
  Python과 C의 좌우 핀 매핑·기구학·속도/PWM 제한, 모의 AUTO 357/784, 영상/압력 기록을 포함한다.
- 후처리 시각 경계·crop 좌표·관측 불가 관절·feature 부호 시험 **5개 통과**.
- 독립 압력 파서의 두 정의는 원본 `seat12_core.py`와 AST가 같음을 확인했다.
  uint32 wrap·누락·중복·reset과 오류 ADC 표본의 빈 raw 필드를 별도로 시험했다.
- GUI만 저장소 밖 임시 폴더로 복사해 import·기본 창 생성·구동 비활성 초기 상태를 확인했다.
  이 시험은 장치 목록을 mock 처리했으며 실제 COM/카메라를 열지 않았다.
  패키지 내 Markdown 링크와 VS Code workspace JSON도 확인했다.
- ST-LINK serial 없이 Upload를 요청하면 도구/보드 접근 전에 거부되는 것을 확인했다.
- 이 패키징 검증에서는 STM32 업로드와 실제 모터 이동을 실행하지 않았다.
  기존 하드웨어 실측 이력은 [gui/VALIDATION.md](gui/VALIDATION.md)에 보존했다.

소스만 커밋하며 기록 세션·영상·모델·빌드 산출물은 제외한다.
