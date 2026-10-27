# 검증 기록 — 2026-10-25

기준 브랜치 `feat/beamng-pressure-fallback`, 원래 HEAD `d3a95c891585fd84911b74e3256b3563f7d26870`. 변경사항은 로컬 작업 브랜치에 있으며 원격 push는 하지 않았다.

## 결과

| 검사 | 결과 |
|---|---|
| Python 단위·통합 | **73개 PASS** |
| 실제 fallback ELF 에뮬레이션 | **21개 PASS** |
| 압력 전용 ELF 회귀 | **5개 PASS** |
| 기본 통합 펌웨어 빌드 | PASS |
| 압력 전용 펌웨어 빌드 | PASS, 기존 배포 BIN과 바이트 동일 |
| fallback/pressure-only 동시 선택 거부 | PASS |
| ELF PT_LOAD와 BIN 일치 | PASS |
| BeamNG 수집기 v0.3 읽기 전용 API 검사 | PASS, 설치된 소스 해시 변경 없음 |
| git diff --check | PASS |

호스트 시험에는 부호 전환·중립 경유·각도/속도 상한·압력 초과·입력 지연·MCU/stream 재시작·ACK 불일치·재전송 금지·OFF 확인·다시 ARM하지 않는 동작이 포함된다. 실제 `run_live`를 가짜 UART·소켓·시계로 실행한 9개 통합 시험에서 시작 데이터 2초 + 중립 기준선 2초, 늦게 시작한 압력, 한 배치의 invalid→valid 입력도 검사했다. 합성 압력 응답은 실제 압력 성능의 증거로 사용하지 않았다.

ELF 시험은 생성된 ARM 코드를 실행한다. 레지스터/통신 시험은 M7 모델, double 명령 한계가 있는 수학/연속 추종은 ARM_MAX와 PWM 경계 stub을 사용한다. 실제 파형·축 위치·토크·하중·최악 지연을 검증했다는 뜻이 아니다. 상세 구성과 한계는 [펌웨어 계약](../stm32/FALLBACK_PROTOCOL.md)을 참고한다.

## 기록 파일 재생

주행 CSV `session_20261025_061416_car_A_free_cbfe254c/vehicle/vehicle.csv`:

- 총 13,223행, 정상 주행 입력 10,957행, 정지 2,233행.
- host 시간 공백 2행, 횡가속도 이상값 21행, 종가속도 이상값 7행, 미분 무효 3행.
- 원본 SHA256 `f8a56797a8e43d56a6453062da5d9d3bfaa4081c10a67a1cd3e421e12e5e082c`.
- 정지 기록 5,219행도 검사했다. 지지 동작을 요구하는 유효 주행 입력으로 분류하지 않았다.

재생은 시간·품질·요구량·후보 각도 분석이다. 제어 루프에 실측 압력을 동시 입력한 실험이나 인체/기구 동역학 시뮬레이션은 아니다. 실제 하드웨어 명령은 **0개**다.

## 데이터·산출물

- bench 프로파일 SHA256: `898bc5ecb825efc7f3a7b162d9812940d98e99d845805b3ca4d6f6b738fb915d`.
- fallback ELF SHA256: `78f51982e8cbe31008c58c76a6260c3f52f6ae27f13b5b13bdb36e506bc6f212`.
- fallback BIN SHA256: `2e4045b79edc5a302c0d7a76585e3e9c378ee72b65c5c6c31015029123b77cdc`.
- 로컬 호스트 시험 로그: 로컬 빌드 폴더의 `fallback_review/host_tests_final.log`.
- 로컬 최종 차량 재생: 같은 폴더의 `vehicle_replay_final/report.json`, CSV, PNG. report에 전체 설정·입력·프로파일·소스 해시 포함.
- 펌웨어 생성물/검증 영수증: `../stm32/build_fallback/`.

압력 프로파일은 영상 없이 55개 명령점·132,660 ADC 표본에서 만들었다. 사용자 센서 매핑 적용 후 224개 채널 평균을 원시 압력 CSV와 교차검증했다. 세션간 부호 변화는 [mapped_analysis.json](profiles/mapped_analysis.json) 및 [노트](profiles/bench_20261025_NOTES.md)에 기록했다.

## 수행하지 않은 것

이번 작업에서 실제 UART/보드/모터 접근, 펌웨어 업로드, 설치된 수집기 실행·변경, 실제 UDP 포트 바인드, 영상 분석은 하지 않았다. 기존 압력 전용 실행 환경은 유지했다. 실제 착좌 기준선·지지 동작·압박감·파형·시간 지연·효과 비교는 다음 하드웨어 시험 범위다.
