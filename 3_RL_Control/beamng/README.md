# BeamNG 연결부 (BeamNG_Bolster_FB 경로 → Raspberry Pi)

BeamNG.drive 차량 데이터는 `BeamNG_Bolster_FB`(Experiment A V6)와 같은 경로로 받습니다. PC는 그 데이터를 Pi로 넘기기만 하고, 제어는 Pi가 합니다.

```
PC (Windows)
  BeamNG.drive ──UDP 64256──► BeamNG Collector v0.3        (외부 프로그램, 변경 없음, 원본 CSV/JSONL 보관)
                                 └ python -m fallback.beamng_bridge --collector-root <Collector 폴더>
                                     (4_Seat_Actuation/, FB 코드 그대로; 행마다 사본 → 127.0.0.1:64257)
  python "3_RL_Control/beamng/pi_relay.py" --pi <PI_IP>   (이 폴더; 127.0.0.1:64257 → <PI_IP>:64257)
                                         │  유선 LAN 권장
Raspberry Pi 5                           ▼
  python -m oasis_rt.app                 수신 → FB VehicleValidator → RL / 규칙 / FB V6 → STM32 (OASIS_FALLBACK v2)
```

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `pi_relay.py` | PC | FB 브리지 데이터그램을 **바이트 그대로** Pi로 전달합니다. 앞에 `OASIS-RELAY1 <pc_age_ns> <seq>` 헤더 한 줄을 붙입니다. |
| `fake_collector.py` | PC (또는 Pi) | BeamNG 없이 시험할 때 Collector + 브리지 대신 합성 행을 보냅니다. `source_kind = synthetic_test`이므로 Pi에서 `beamng.allow_synthetic: true`일 때만 받습니다. |

## 신호 계약 (BeamNG Collector v0.3, revision 2)

| 항목 | 계약 |
|---|---|
| 좌표 | 차량 +x 전방, **+y 왼쪽**, +z 위 |
| `linear_acc_y_mps2`, `linear_acc_x_mps2` | 중력 제외(속도 미분, Collector 계산), m/s² |
| `speed_mps` | m/s. 1 m/s 미만은 `stopped` (볼스터 열림) |
| `steering_angle_deg` | **핸들 각**(차륜각 아님), deg. 부호는 `beamng.steer_sign`으로 맞춥니다(아래 확인 절차) |
| `throttle_input`, `brake_input`, `gear_index`, `roll_rad`, `pitch_rad` | FB V6의 2° 선제 조건에만 씁니다 |
| 요레이트 | Collector에 없습니다. Pi는 a_y / max(v, 5 m/s)로 추정합니다(정상 원선회 관계) |
| 품질 | FB `VehicleValidator`: `config_match`, `derivative_valid`, 순번·stream·session 연속, 행 나이·간격 ≤ 200 ms, \|a_x\|, \|a_y\| ≤ 200 m/s²(무결성). 충격 의심(FB V7 `ImpactDetector`: 평면 가속도 ≥ 25 m/s²가 0.06 s 이상 + 급변 ≥ 150 m/s³ + 구간 적분 ≥ 1.5 m/s, 또는 ≥ 60 m/s² 즉시) → OFF |
| 시계 | Collector는 PC `perf_counter_ns`로 수신 시각을 찍습니다. 릴레이가 PC 쪽 경과(`pc_age_ns`)를 재고, Pi는 `수신 시각 − pc_age`로 다시 찍어 200 ms 신선도를 Pi 시계로 검사합니다. 네트워크 전송 시간(유선 ~1 ms)만 빠집니다 |
| 오류 | ARM 뒤의 잘못된 행, 끊김(> 200 ms), 순서 어긋남, 충격 의심, 조향 부호 불일치 → **FB_OFF, 세션 종료, 자동 재ARM 없음** (FB와 같음) |

BeamNG 메뉴, 일시정지, 차량 로딩으로 200 ms 넘게 끊겨도 세션이 끝납니다. FB V5에서 확인한 대로 게임은 **Borderless** 창 모드로 둡니다(전체화면 전환 때 0.6 s 공백이 생깁니다).

## 시작 순서 (실기)

1. **PC: BeamNG.drive**를 Borderless로 실행합니다. FB와 같은 차(순정 Vivace 160 DCT)를 고르고 주행 상태로 둡니다.
2. **PC: Collector + 브리지**를 실행합니다. `4_Seat_Actuation`에서 다음을 실행하세요.
   `python -m fallback.beamng_bridge --collector-root C:\path\to\BeamNG_Collector_v0_3`
   - Experiment A 바탕화면 앱(`BeamNG Bolster A`)과 동시에 띄우지 않습니다. 둘이 같은 포트를 씁니다.
3. **PC: 릴레이**를 실행합니다.
   `python "3_RL_Control/beamng/pi_relay.py" --pi 192.168.0.50`
   - 5 s마다 전달 수, 초당 행 수, PC 쪽 지연 p50/p95를 출력합니다.
4. **Pi: 스트림 확인** (STM32 없이). 주행하면서 좌·우 회전을 몇 번 합니다.
   `python -m oasis_rt.app --check-vehicle 30`
   - 행 수와 비율을 확인합니다.
   - 간격 p95/최대를 확인합니다.
   - `steer_verdict`가 `ok`인지 확인합니다. `mismatch`이면 `config.yaml beamng.steer_sign: -1`로 바꾸고 다시 확인합니다.
   - 확인이 끝나면 `confirmations.steer_sign_checked: true`로 둡니다.
5. **Pi: 세션 시작**: `python -m oasis_rt.app` (또는 `systemd`). 절차는 [`deploy/README.md`](../deploy/README.md)에 있습니다.

PC 직결 LAN에서 IPv6 link-local 주소를 쓸 때도 릴레이를 사용할 수 있습니다.
PC의 `--pi`에는 `fe80::주소%인터페이스번호`를 넣고, Pi 설정은
`beamng.listen: ["::", 64257]`, `beamng.allow_from: ["PC의 fe80::주소"]`로 둡니다.
scope 번호는 각 컴퓨터의 로컬 인터페이스 번호이므로 서로 복사하지 않습니다.
IPv6 수신기는 IPv6만 받으며 기존 IPv4 기본 설정은 유지됩니다.

## BeamNG 없이 확인하기

```bash
# PC 한 대에서 끝까지 (Pi 대신 같은 PC에서 config의 beamng.allow_synthetic: true로 --check-vehicle 실행)
python beamng/fake_collector.py --seconds 120          # → 127.0.0.1:64257
python beamng/pi_relay.py --pi <PI_IP>                 # → Pi
# Pi: beamng.allow_synthetic: true 로 두고
python -m oasis_rt.app --check-vehicle 30
```

장치를 전혀 쓰지 않는 SIL도 있습니다. 합성 행과 OASIS_FALLBACK v2 모의 보드를 모의 시계에서 돌립니다.
`python -m oasis_rt.app --sil 60`

카메라 pose는 BeamNG 릴레이에 섞지 않습니다. `oasis_rt.app`의 기본값은 `pose.source: none`이며, 향후 같은 Pi의 전처리 feature만 받는 선택형 loopback `oasis.pose.v1` 계약은 [배포 README](../deploy/README.md#향후-카메라-pose-입력--현재-기본값은-off)에 있습니다. 이 인터페이스를 준비해도 카메라·pose 수신을 자동으로 켜지 않습니다.

## 선택 입력: MOZA 직접 조향

`moza_sender.py`는 Windows WinMM으로 MOZA R3(346e:0005)의 X축을 읽기만 합니다. 게임 포커스를 요구하지 않으며 FFB·기기 설정·모터 명령을 사용하지 않습니다. 원래 Collector 기록과 차량 릴레이는 유지하고 `oasis.moza.v1`을 별도 UDP 64258로 보냅니다.

```bash
python beamng/moza_sender.py --pi <PI_IP> --seconds 120 --log <새_JSONL_경로>
```

Pi `wheel.source: udp`에는 PC IP 허용 목록과 확인된 게임 각도 변환을 지정합니다. `wheel.priority_prediction: true`일 때만 유효한 직접 조향을 wheel preview에 우선 사용하고, 누락·단절·지연·미확인 보정이면 원래 BeamNG 예측기를 사용합니다. 측정 횡/종가속도와 속도, 차량 신선도·충격 판정, STM/CAD 제한은 그대로입니다. 게임 조향 부호(`beamng.steer_sign`)는 두 경로에 동일하게 적용됩니다.

현재 사용자가 확인한 Pit House 물리 회전 범위는 총 450°(±225°)입니다. 원시 정규화 축, 물리 각도, 게임 `electrics.steering` 각도는 별개입니다. 기본 게임 입력맵의 720°만으로 배율을 결정하지 않습니다. `game_gain_deg`와 `game_offset_deg`는 원시 축과 실제 Collector 조향의 비교 자료로 정합니다. PC 타임스탬프는 USB 하드웨어 시각이 아닌 호스트 조회 시각이며, 절대 네트워크 전달 지연이나 예측 성능 향상을 측정했다고 주장하지 않습니다.

2026-10-27 두 기록의 원시 패킷 5,081개를 대조한 결과, WinMM 정규화 축은 게임 `steering_input`과 양방향에서 일치했습니다(RMSE 0.000262 / 0.000384). 현재 차량 `52076`의 입력 기반 게임 핸들 명령 좌표는 `game_gain_deg: -560`, `game_offset_deg: 0`으로 준비했습니다. 이는 필터 지연이 있는 순간 `electrics.steering` 실측각과 구별합니다. 후보 설정에는 `calibration.beamng_vehicle_id: '52076'`를 지정하고 다른 차량에서는 기존 예측기로 돌아갑니다. 게임 입력 설정을 바꾸면 다시 대조해야 합니다.

실제 PC→Pi 읽기 전용 시험에서는 MOZA 998개 패킷이 모두 수신·검증됐고, BeamNG 602행을 함께 받았습니다. 정차 상태의 예측 경로 전환까지 확인했으며 모터 명령은 보내지 않았습니다. 주행 중 예측 정확도와 모터 반응 효과는 후속 시험 대상입니다.

## 검증 상태

| 시험 | 결과 |
|---|---|
| fake_collector → pi_relay → Pi 수신기 (localhost, 실제 UDP) | 600행 전달, 손실 0. 첫 행만 `first_sample`(FB 규칙). PC 쪽 지연 p95 0.14 ms. 조향 부호 `ok` |
| 릴레이 헤더, Pi 재타임스탬프, 깨진 데이터그램 거부 | `deploy/runtime/tests/test_app_fb.py` 통과 |
| FB 모듈 자체 시험 (`experiment_a`, `fallback`에서 main에 넣은 부분) | 71 + 46개 통과 |
| 실제 BeamNG.drive + Collector v0.3 + 핸들 | **이 PC에는 BeamNG가 없어 미시험.** FB 쪽 기록: 같은 Collector 경로로 V6가 차량 행과 압력을 받고 중립 연결까지 확인했습니다. 주행 구동은 미시험입니다(`4_Seat_Actuation/experiment_a/README.md`) |

## 이전 경로 (main에 넣지 않음)

`feat/rl-final-beamng` 브랜치에 그대로 남아 있습니다.
- `bridge.py`: BeamNG.tech + BeamNGpy 도로 미리보기, BeamNG.drive MotionSim/OutGauge + SDL 핸들
- `fit_wheel.py`, `fit_vehicle.py`, `record_sequences.py`

v8 예측기는 Collector 입력(도로 없음, 요레이트 = a_y / max(v, 5))으로만 학습했습니다. 1 s 오차는 0.090 g입니다(hold 0.140). 도로 미리보기가 있던 v7 예측기는 0.069 g였습니다(`deploy/CONTROL_SPEC.md` §3).
