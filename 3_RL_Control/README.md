# 3_RL_Control — O.A.S.I.S 능동 볼스터 RL 제어 (최종 v8, Raspberry Pi + BeamNG.drive)

담당은 RL-001(김병준)입니다. 좌·우 사이드 볼스터의 yaw 목표각을 정하는 제어기와, 그 학습·검증 환경, Raspberry Pi 런타임이 들어 있습니다.

```text
PC : BeamNG.drive → Collector v0.3 → fallback.beamng_bridge (../4_Seat_Actuation) → beamng/pi_relay.py
Pi : python -m oasis_rt.app (deploy/runtime) → STM32 NUCLEO-H723ZG, OASIS_FALLBACK v2 (../4_Seat_Actuation/stm32)
```

- **제어기**
  - RL(기본): 튜닝 규칙 위의 SAC 잔차 정책입니다.
  - 액터가 실패하면 그 튜닝 규칙으로 전환합니다(hot fallback).
  - 모델 패키지가 없으면 BeamNG_Bolster_FB의 Experiment A 제어기(`fb_v6`, V6 제어 법칙)로 돕니다(cold fallback).
- **BeamNG 연결, 보드 프로토콜, CAD 한계, 세션 규칙**은 `BeamNG_Bolster_FB`(`../4_Seat_Actuation`)를 그대로 씁니다.
- **v8**은 그 보드의 실제 구동기(서보당 2 µs/20 ms)와 Collector 입력(도로·요레이트 없음)으로 다시 학습한 모델입니다.
  - 튜닝 규칙에서 출발해 ±0.25 범위에서 조정합니다.
  - 보상은 평가 점수 그 자체입니다. v7 보상은 규칙을 볼스터 열림보다 낮게 평가했습니다([`CONTROL_SPEC.md` §5.1](deploy/CONTROL_SPEC.md)).
- **10/28 착석 시험**은 G 기본 곡선 + v8 actor의 ±1.5° 보정으로 돌았습니다(`deploy/experiments/seated_20261028`).
- **v9**는 그 구조를 고정하고, 다른 사람이 앉을 때 쓰는 체형 스케일만 더했습니다([§4b](deploy/CONTROL_SPEC.md)).
  - 카메라 중립의 어깨폭 비를 기준 사용자(10/28 최종 세션)와 비교해, G 기본각을 0.9–1.1배 합니다.
  - 기본은 OFF입니다(s = 1).

| 문서 | 내용 |
|---|---|
| [`deploy/README.md`](deploy/README.md) | **Pi 설치·운용**: 모델 패키지, STM 튜닝, 세션 순서, 실패 시 동작 |
| [`deploy/CONTROL_SPEC.md`](deploy/CONTROL_SPEC.md) | 제어 사양: 관측·행동, BeamNG 예측기, 튜닝 규칙, RL, 안전 계층, 세션과 fallback, 결과 |
| [`beamng/README.md`](beamng/README.md) | PC 쪽 연결 (Collector → 릴레이 → Pi), 신호 계약, BeamNG 없이 시험하기 |
| [`sim/v3/MODEL_CARD_v8.md`](sim/v3/MODEL_CARD_v8.md) | 최종 모델 카드 (검증, 결과, 한계) |
| [`sim/v3/final/PARAMS_FINAL.md`](sim/v3/final/PARAMS_FINAL.md) | 가설과 맞는 데이터로 확정한 인체·압력·포즈·차량·구동기 변수와 DR 범위 |

## 구조

```text
sim/                 인체 모델, 시나리오(BeamNG급 동적 차량), 센서; v2 → v3 환경 (env_v3 = 최종 환경)
  v2/firmware_model  C4_R8 IK, OASIS_CONTROL v6 경로, OASIS_FALLBACK v2 프레임 (fb_a2_step)
  v3/overlays/       최종 환경 overlay 체인 (overlays.txt 순서로 누적 적용): ... final_v7 + fb_a2 (보드 구동기) + beamng_drive_collector (Pi 입력) + reward_score (보상 = 평가 점수)
  v3/reward_diag.py  보상 항별 분해: 보상과 평가 점수가 제어기를 같은 순서로 보는지
  v3/tune_body_scale.py, reference_body_from_video.py   v9 체형 스케일 튜닝·홀드아웃 검증, 기준 사용자 영상 측정
  v3/final/          curate.py, train_predictor.py, run_final.sh (전체 재현), env.sh, overlays.txt
  rl/                SAC (sac_asym.py), 평가(evaluate.py, FB V6 포함), 체크포인트 선택, export
deploy/
  runtime/oasis_rt/  app.py(Pi 진입점), fb_link, beamng_rx, envelope, controller, actor(+C), predictor, safety,
                     g_residual_control (10/28 G 기본 곡선 + RL 보정), body_scale (v9)
  runtime/tests/     test_app_fb.py (OASIS_FALLBACK v2 모의 보드 SIL), test_runtime.py
  export/, tools/    모델 패키지 만들기 (export_actor, make_calib_set, pack_model)
beamng/              pi_relay.py (PC → Pi), fake_collector.py (BeamNG 없이 시험)
results/v9/          기준 사용자 체형(영상 측정), 체형 스케일 튜닝·홀드아웃 검증
results/v8/          최종 정책: 규칙, 감사, 예측기 평가, 시드 선택, 평가 요약, 시험 결과 (JSON / 텍스트)
results/v7/          최종 시뮬레이터 파라미터 선별 근거 (curate.py 출력: curation, corr)
```

## 재현

```bash
source sim/v3/final/env.sh          # 최종 환경 (보정 calib_v7.json, overlay sim/v3/final/overlays.txt)
bash sim/v3/final/run_final.sh      # 예측기 → 감사 → 규칙 → SAC → 검증 선택 → 평가 → export → 시험 → 관측 제거 → 체형
python deploy/tools/pack_model.py deploy/model_v8          # Pi용 패키지 (dist/, git 밖)
python sim/v3/tune_body_scale.py --runtime deploy/runtime --out results/v9/body_scale.json --csv <episodes.csv>   # v9 (12코어 약 15분)
python sim/v3/tune_body_scale.py --runtime deploy/runtime --validate -1.2 0 0.9 1.1 --out results/v9/body_scale_validation.json
```

- 원시 데이터, `models/`, `deploy/model*/`, 체크포인트(`*.pt`, `*.npz`)는 저장소에 포함하지 않습니다.
- `models/v5/l1_seq.csv`는 실차 원본에서 `python sim/v2/make_l1_seq.py`로 만듭니다.
- v8은 4 시드를 GPU에서 학습했습니다. CUDA가 있는 torch를 가진 Python을 지정합니다.
  ```bash
  OASIS_TRAIN_PY=<cuda torch python> OASIS_SEEDS="0 1 2 3" OASIS_TRAIN_THREADS=2 bash sim/v3/final/run_final.sh
  ```
  - 시드당 200k 스텝, 약 1시간(RTX 4070 SUPER, 동시 4개)입니다.
  - 기본값은 CPU Python, 2 시드 × 5 스레드입니다. 시드당 약 1시간이고, 전체는 12코어 PC에서 약 2시간입니다.

## 최종 결과 (평가 은행 48 에피소드, 같은 에피소드 비교, OASIS_FALLBACK v2 구동기)

| 제어기 | J | CoM 감소 L0 / L2 | 20 N 초과 시간 L0 / L2 | 규칙 대비 같은 에피소드 차 [95 % CI] | 이긴 에피소드 |
|---|---|---|---|---|---|
| **RL `v8_s1` (배포)** | **3.92** | 8.1 / 10.1 % | 4.8 / 6.9 % | −0.95 [−3.06, +0.73], 중앙값 +0.02 | 32 / 48 |
| **튜닝 규칙 (hot fallback)** | **3.93** | 9.8 / 10.8 % | 4.9 / 7.5 % | — | — |
| FB V6 (cold fallback) | 1.70 | 3.5 / 4.4 % | 2.2 / 2.9 % | −2.34 [−6.81, +2.10] | 10 / 48 |

- RL과 규칙은 같은 수준이며, 백업 제어기(FB V6)보다는 분명히 낫습니다.
- 4 시드 중 2개는 평가 은행 꼬리에서 크게 잃었습니다(J 2.97, 2.29). 자세한 내용은 [`CONTROL_SPEC.md` §6](deploy/CONTROL_SPEC.md)에 있습니다.

시뮬레이터 감사 13/13, 런타임 시험이 통과했습니다. 2026-10-28에 BeamNG.drive + STM32 보드 + Pi + 실제 착석자로 실물 시험을 수행했습니다(G 기본 곡선 + RL 보정, [`deploy/experiments/seated_20261028`](deploy/experiments/seated_20261028/README.md)).
