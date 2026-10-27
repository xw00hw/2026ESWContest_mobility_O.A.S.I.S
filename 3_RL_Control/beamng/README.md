[← 3. RL 제어](../README.md) &nbsp;|&nbsp; **① 센싱** → ② 처리 → **③ RL 제어** → ④ 시트 구동

# PC ↔ Raspberry Pi 연결 (beamng)

> 주행 시뮬레이터(BeamNG.drive)의 차량 데이터와 조향 휠 입력을 Raspberry Pi로 넘기고, Pi의 제어 상태를 PC에서 실시간으로 봅니다.
> **PC는 데이터를 전달만 하고, 제어 결정은 모두 Pi가 합니다.**

---

## 데이터 경로

```text
PC (Windows)
  BeamNG.drive ──UDP 64256──► BeamNG Collector v0.3        (외부 프로그램, 원본 CSV/JSONL 보관)
                                 └─ fallback.beamng_bridge  (4_Seat_Actuation/fallback, 행마다 사본 → 127.0.0.1:64257)
  pi_relay.py   127.0.0.1:64257 ──────────────────────────────► Pi:64257   (유선 LAN 권장)
  moza_sender.py  MOZA R3 휠 축 ────────────────────────────────► Pi:64258   (선택)
  rl_monitor.py   ◄── Pi control.jsonl (SSH, 읽기 전용) ── http://127.0.0.1:8766

Raspberry Pi 5
  python -m oasis_rt.app   수신 → 입력 검증 → RL / 규칙 / 백업 제어 → STM32
```

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `pi_relay.py` | PC | 브리지 데이터그램을 **바이트 그대로** Pi로 전달. 앞에 `OASIS-RELAY1 <pc_age_ns> <seq>` 헤더를 붙여 PC 쪽 지연을 알립니다 |
| `moza_sender.py` | PC | MOZA R3 휠의 조향 축을 읽기만 해서 Pi로 보냅니다 (FFB·장치 설정 변경 없음) |
| `rl_monitor.py` | PC | Pi 제어 로그를 읽어 G·기본각·RL 보정·전송 목표·보드 명령·압력·카메라 상태를 그래프로 표시 (읽기 전용) |
| `fake_collector.py` | PC / Pi | BeamNG 없이 시험할 때 합성 주행 데이터를 보냅니다 |

## 신호 계약

| 항목 | 정의 |
|---|---|
| 좌표 | 차량 +x 전방, **+y 왼쪽**, +z 위 |
| 횡·종가속도 | 중력 제외, m/s² |
| 속도 | m/s. 1 m/s 미만은 정지로 보고 볼스터를 엽니다 |
| 조향 | **핸들 각**(바퀴 각 아님), deg. 부호는 `beamng.steer_sign`으로 맞춤 |
| 요레이트 | Collector에 없어 `a_y / max(v, 5 m/s)`로 추정 (학습 시뮬레이터도 같은 식) |
| 신선도 | PC 쪽 경과 시간을 헤더로 보내고, Pi가 자기 시계로 다시 찍어 **200 ms** 신선도를 검사 |
| 품질 | 순번·스트림·세션 연속성, 행 간격 ≤ 200 ms, 수치 무결성, 충격 의심 판정 |
| 오류 시 | ARM 이후 잘못된 행·끊김·순서 어긋남·충격 의심·조향 부호 불일치 → **PWM OFF, 세션 종료, 자동 재시작 없음** |

게임 메뉴, 일시정지, 차량 로딩으로 200 ms 넘게 끊겨도 세션이 끝납니다. 전체화면 전환 때 공백이 생기므로 게임은 **Borderless** 창 모드로 둡니다.

## 실행 순서

1. **PC:** BeamNG.drive를 Borderless로 실행하고 주행 상태로 둡니다.
2. **PC:** Collector + 브리지 실행 (`4_Seat_Actuation`에서)
   ```powershell
   python -m fallback.beamng_bridge --collector-root C:\path\to\BeamNG_Collector_v0_3
   ```
3. **PC:** 릴레이 실행 — 5초마다 전달 수, 초당 행 수, PC 쪽 지연 p50/p95를 출력합니다.
   ```powershell
   python 3_RL_Control/beamng/pi_relay.py --pi <PI_IP>
   ```
4. **Pi:** 수신 확인 (보드를 열지 않음). 주행하며 좌·우 회전을 몇 번 하고 `steer_verdict`가 `ok`인지 봅니다.
   ```bash
   python -m oasis_rt.app --check-vehicle 30
   ```
   `mismatch`이면 `config.yaml`의 `beamng.steer_sign`을 `-1`로 바꾸고 다시 확인합니다.
5. **Pi:** 세션 시작 — [`deploy/README.md`](../deploy/README.md#세션-실행)
6. **PC (선택):** 모니터
   ```powershell
   python 3_RL_Control/beamng/rl_monitor.py --ssh-host <PI_HOST> --ssh-log /absolute/run/control.jsonl --open-browser
   ```

## 조향 휠 직접 입력 (선택)

`moza_sender.py`는 Windows WinMM으로 MOZA R3의 조향 축을 읽어 UDP 64258로 보냅니다. 게임 포커스가 필요 없고 장치 설정이나 모터 명령을 쓰지 않습니다.

```powershell
python 3_RL_Control/beamng/moza_sender.py --pi <PI_IP> --seconds 120 --log <새 JSONL 경로>
```

Pi에서 `wheel.priority_prediction: true`이고 보정이 확인된 경우에만 휠 입력을 횡가속도 예측에 우선 사용합니다.
휠 입력이 없거나 끊기거나 늦으면 원래 예측기로 돌아갑니다. 휠 축은 게임의 조향 입력과 RMSE 0.0003 수준으로 일치함을 확인했습니다.

## BeamNG 없이 시험하기

```bash
python beamng/fake_collector.py --seconds 120     # 합성 주행 데이터 → 127.0.0.1:64257
python beamng/pi_relay.py --pi <PI_IP>            # → Pi
python -m oasis_rt.app --check-vehicle 30         # Pi: config의 beamng.allow_synthetic: true 필요

python -m oasis_rt.app --sil 60                   # 장치·네트워크 없이 모의 보드로 세션 전체
python -m unittest discover -s beamng/tests       # 모니터 시험
```

| 시험 | 결과 |
|---|---|
| 합성 데이터 → 릴레이 → Pi 수신기 (실제 UDP) | 600행 전달, 손실 0, PC 쪽 지연 p95 0.14 ms |
| 릴레이 헤더, 재타임스탬프, 깨진 데이터 거부 | `deploy/runtime/tests/test_app_fb.py` 통과 |
| 휠 입력 → Pi (실제 네트워크, 모터 없이) | 998개 패킷 모두 수신·검증, 예측 경로 전환 확인 |
