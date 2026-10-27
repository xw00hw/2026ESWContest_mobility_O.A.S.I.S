[← 4. 시트 구동](../README.md) &nbsp;|&nbsp; **① 센싱** → ② 처리 → ③ RL 제어 → **④ 시트 구동**

# 주행 시뮬레이터 브리지 · 규칙 기반 위치 제어 (fallback)

> 주행 시뮬레이터(BeamNG.drive)의 차량 데이터를 받아 검증하는 **브리지**와, 횡가속도에 비례해 볼스터를 움직이고 압력 증가를 감시하는 **규칙 기반 제어기**입니다.
> 최종 시스템에서는 브리지(`beamng_bridge.py`)가 PC의 데이터를 Raspberry Pi로 넘기는 출발점입니다.

---

## 최종 시스템에서의 역할

```text
BeamNG.drive ──UDP 64256──► Collector v0.3 (원본 저장)
                                └─► beamng_bridge.py ──UDP 127.0.0.1:64257──► 3_RL_Control/beamng/pi_relay.py ──► Pi
```

| 파일 | 역할 |
|---|---|
| `beamng_bridge.py` | 기존 Collector의 저장 경로를 바꾸지 않고, 행마다 제어용 사본을 UDP로 전달 |
| `beamng.py` | 메시지 형식(schema `oasis.beamng.fallback.v1`), 시각·단위·순서·품질 검사. Pi 런타임도 import |
| `serial_link.py` | 압력 12채널 수신, OASIS_FALLBACK 펌웨어 확인·명령·ACK. Pi 런타임도 import |
| `controller.py`, `profile.py` | 규칙 기반 위치 제어 + 압력 감시 (아래) |
| `runner.py`, `replay.py` | 실제 연결 실행 / 장치 없이 차량 CSV 재생 |
| `profiles/` | 벤치 실측(축 단독 명령 55개, ADC 표본 132,660개)으로 만든 압력 프로파일과 분석 노트 |
| `tests/` | 가짜 소켓·UART·시계와 실제 기록 파일을 이용한 시험 |

## 규칙 기반 제어 방식

```text
1. 수신 시각·시뮬레이션 시각·품질 플래그 검사
2. 횡가속도 0.20 s 저역통과, 0.03 g 켜짐 / 0.02 g 꺼짐
3. 요구량 = clip((|a_y| − 0.02 g) / (0.30 g − 0.02 g), 0, 1) → 밀릴 쪽 Yaw 목표 = 요구량 × 20°, 반대쪽 0°, Pitch 0°
4. 목표 변화율 최대 2 °/s, 방향 전환 시 중립 경유
5. 착석 직후 측정한 중립 압력 대비 채널별 증가 감시
     기준 초과가 0.1 s 지속 → 중립으로 복귀
     기준의 2배 즉시 초과 → PWM OFF 고정
```

- 차량 좌표는 +x 전방, +y 왼쪽입니다. a_y > 0이면 몸이 오른쪽으로 밀리므로 오른쪽 볼스터를 닫습니다.
- 압력이 작다고 각도를 계속 올리는 적분 제어는 하지 않습니다. 같은 각도에서도 세션마다 압력 변화의 부호가 달라, 압력 → 각도 역산이나 N 단위 환산은 만들지 않았습니다.

### 측면 압력 센서 위치 (착석자 기준)

| 센서 | 위치 |
|---|---|
| A1_C0 | 오른쪽 아래 |
| A1_C1 | 오른쪽 위 |
| A1_C2 | 왼쪽 아래 |
| A1_C3 | 왼쪽 위 |

## 실행 방법

```powershell
# 4_Seat_Actuation 폴더에서 — 아래 명령은 포트·장치를 열지 않습니다
python -m fallback --config fallback/config.example.json inspect
python -m fallback --config fallback/config.example.json replay --vehicle-csv <vehicle.csv> --out <새 폴더> --plot
python -m fallback.beamng_bridge --collector-root <BeamNG_Collector_v0_3> --check-only

# 브리지 실행 (PC)
python -m fallback.beamng_bridge --collector-root <BeamNG_Collector_v0_3>

# 규칙 제어기 단독 실물 실행 (설정·포트·--arm이 모두 있어야 구동)
python -m fallback --config <검토한 설정.json> run --port COM11 --arm --out <새 폴더>
```

## 신호·시간·오류 계약

| 경로 / 항목 | 계약 |
|---|---|
| BeamNG → Collector | UDP 127.0.0.1:64256, Collector 하나만 수신·원본 저장 |
| Collector → 브리지 사본 | UDP 127.0.0.1:64257, 원래 `host_monotonic_ns` 유지 |
| 시간 | 같은 PC의 `perf_counter_ns`, 벽시계로 다시 찍지 않음 |
| 신선도 | 차량·보드 상태 최대 200 ms, 압력 UART 최대 100 ms |
| 압력 | C0~C3가 모두 이어진 완전한 스캔만 사용, 오래된 채널 재사용 금지 |
| UART | 460800 bps, 요청 하나만 대기, 응답 200 ms 제한 |
| 모터 lease | 단일 제어 루프가 건강한 판단을 내렸을 때만 PING. 압력 PING은 모터 유효시간을 갱신하지 않음 |
| 오류 | 입력 지연·이상·재시작·ACK 실패·압박 초과 → OFF 고정, 자동 재ARM 없음 |

실행 기록은 `manifest.json`, `control.jsonl`에 남고, Collector 원본 저장은 그대로 이어집니다.

## 검증

```powershell
python -m unittest discover -s fallback/tests -q
```

단위·통합 시험, 가짜 UART·소켓·시계, 실제 기록 CSV 재생, 빌드된 펌웨어 ELF의 레지스터·프로토콜·추종 에뮬레이션까지 검증했습니다. 상세: [`VALIDATION.md`](VALIDATION.md)
