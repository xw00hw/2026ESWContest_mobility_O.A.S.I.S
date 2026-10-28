[← 보드 통신 · 차량 입력 검증 · 백업 제어기](README.md)

# 몸통 응답 모델 (human_model)

> `human_model.py`는 시뮬레이터 차량 가속도에 대한 **보조 없는 가상의 몸통 응답**을 예측하는 2축 질량–스프링–댐퍼 모델이고, `alignment.py`는 예상 기울기에 볼스터 패드 면을 맞추는 Pitch를 계산합니다.
> 착석자를 관측하거나, 볼스터 보조 후 결과를 예측하거나, 접촉력을 추정하지 않습니다.

---

## 1. 현재 시스템에서의 위치

| 모듈 | 현재 백업 제어기(V7)에서의 사용 |
|---|---|
| `human_model.py` | **사용하지 않음.** 오프라인 설계·민감도 분석용 모듈로 유지하며 `tests/test_human_model.py`로 시험 |
| `vehicle_response.py` | 사용. 저역통과한 차량 가속도의 벡터 각도 `atan2(a_y, g)`만 계산. 체형·압력·카메라 입력 없음 |
| `alignment.py` | 사용. 위 차량 가속도 각도를 패드 면 정렬의 기울기 신호로 사용 |

이전 후보 설정은 이 몸통 모델의 예상 기울기를 패드 면 정렬에 사용했으나, 현재는 체형 보정 없이 차량 가속도 벡터만 사용합니다(`participant_source = none_no_body_compensation`). 후보 변천은 [VALIDATION.md](VALIDATION.md#9-대체된-후보-설정)를 참고합니다.

> 실제 시트는 정지해 있으므로 BeamNG 가속도는 착석자에게 실제로 가해진 가속도가 아닙니다. 수치 적분의 정밀도가 생리학적 예측 정확도를 뜻하지는 않습니다.

## 2. 근거 문헌과 한계

| 문헌 | 이 모델에서 쓰는 내용 | 한계 |
|---|---|---|
| [Desai et al., 2023, *Computationally efficient human body modelling for real time motion comfort assessment*](https://arxiv.org/pdf/2306.12279), 2–2.1절, Table 2 | 기준 신장 1.76 m, 질량 75.3 kg 다물체 모델의 인체 측정 관성 파라미터 | 원 모델은 여러 관절·접촉과 실험 적합 파라미터로 검증됨. 2축 축약 모델은 그 검증을 물려받지 **않음**. 원문: “There can be significant individual variation in muscle activation patterns and joint stiffness” |
| [Vette et al., 2014, *Multidirectional quantification of trunk stiffness and damping during unloaded natural sitting*](https://pubmed.ncbi.nlm.nih.gov/24246838/), DOI 10.1016/j.medengphy.2013.10.005 | 건강한 착석자 15명을 8방향으로 교란, 질량–스프링–댐퍼로 기술. 측방과 전방 특성이 다르고 좌우는 대략 대칭 | 방향별 동역학을 분리할 근거일 뿐, 이 시트에 맞는 보편적 강성·감쇠값은 아님. 원문: “both quantities are smallest in the anterior and largest in the lateral directions” |
| [Mirakhorlo et al., 2022, *Effects of seat back height and posture on 3D vibration transmission to pelvis, trunk and head*](https://arxiv.org/html/2207.01989v1) | 등받이 지지 높이와 자세가 진동 전달을 바꿈 | 비교 전 중립 자세·지지 형상·과제를 기록해야 하는 이유. 원문: “Seat back support height and sitting posture substantially affected vibration transmission” |
| [de Leva, 1996, *Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters*](https://pubmed.ncbi.nlm.nih.gov/8872282/) | 인체 측정 질량·질량중심·회전반경을 쓸 때 분절 기준점을 일관되게 정해야 함 | 초록만 확인하고 전체 표는 독립적으로 확인하지 않음. 서로 다른 분절 정의를 섞지 않도록 수치 기본값은 Desai 표에서만 가져옴 |

위 문헌은 1차 연구입니다. 아래의 축약은 문헌의 수식이나 교정값이 아니라 **이 프로젝트의 공학적 가정**입니다.

## 3. 기준 인체 측정값의 축약

Desai Table 2의 머리·목·몸통 5개 분절을 표의 골반 원점 기준으로 하나로 합칩니다. 팔은 상부 몸통에 포함된 것으로 기술되어 있습니다. 표의 좌표(cm)를 m로 바꿨으며 y 좌표는 모두 0입니다. 원저자의 다물체 모델을 재현한 것이 아니라 집중 질량 근사이며, 성별·집단 평균 정확도를 주장하지 않습니다.

| 분절 | 질량 (kg) | 질량중심 Ixx (kg·m²) | 질량중심 Iyy (kg·m²) | x (m) | z (m) |
|---|---:|---:|---:|---:|---:|
| 머리 | 6.23 | .031 | .031 | -.05461 | .69104 |
| 목 | 1.60 | .003 | .004 | -.09420 | .57393 |
| 상부 몸통 | 8.93 | .238 | .146 | -.08231 | .34692 |
| 중부 몸통 | 7.70 | .238 | .146 | -.03827 | .15927 |
| 하부 몸통 | 10.70 | .137 | .078 | .002164 | .05172 |

평행축 정리를 적용하면 다음과 같습니다.

```text
m_ref      = Σ m_i                         = 35.16 kg
h_ref      = Σ m_i·z_i / m_ref             = 0.28729368031854385 m
Iroll_ref  = Σ (Ixx_i + m_i·z_i²)          = 5.447788137970001 kg·m²
Ipitch_ref = Σ (Iyy_i + m_i·(x_i² + z_i²)) = 5.310393044243201 kg·m²
```

전신 질량 `M`, 신장 `H`에 대해 기하학적 상사를 가정한 기본 축척은 `m = M·(35.16/75.3)`, `h = H·(h_ref/1.76)`, `I = m·H²·reference_factor`입니다(`HumanModelConfig`의 `upper_body_mass_fraction`, `com_height_fraction`, `*_inertia_height_factor`로 변경 가능). 이 축척은 개인의 질량 분포, 버팀, 안전벨트, 접촉 유연성, 근육 반응을 결정하지 않습니다.

등받이 각도(`seat_recline_deg`, 수직 기준 뒤로)는 모멘트 팔 근사 `h_vertical = h·cos(recline)`에만 반영합니다. 기울어진 자세에 대해 모드 관성을 다시 식별하지 않으므로, 자세가 크게 바뀌면 모델을 새로 식별해야 합니다. 완전한 3D 인체 측정 변환이 아닙니다.

## 4. 상태 · 부호 · 운동 방정식

| 항목 | 정의 |
|---|---|
| 입력 | 중력을 제외한 BeamNG `ax`, `ay` (m/s²). 차량 좌표 `+x` 전방, `+y` 좌측. 압력·카메라·미래 기록·구동기 응답은 들어오지 않음 |
| 상태 | 가정한 중립 몸통 자세에서의 편차. `roll > 0` = 착석자 오른쪽 기울기, `pitch > 0` = 앞으로 기울기 |
| 입력 부호 | `ay > 0` → 오른쪽 기울기, `ax > 0` → 뒤로 기울기, 제동 → 앞으로 기울기 |

R8 패드의 기구 **Pitch**는 측면 패드의 상·하 돌출 차이를 바꾸는 각도이며, 해부학적 시상면 pitch 상태와는 다릅니다. 구동기 대응은 패드 형상과 지지 방식을 따로 고려해야 합니다.

각 축은 독립된 소각도 모드입니다.

```text
I·q̈ + C·q̇ + K_net·q = m·h_vertical·a        (roll: a = ay,  sagittal pitch: a = −ax)
ω = 2π·f,   K_net = I·ω²,   C = 2·ζ·I·ω
```

`K_net`은 중립 주변의 양의 **순** 복원 계수로, 중력과 기본 자세 유지 효과가 이미 포함되어 있습니다. `m·g·h`를 다시 빼거나 더하지 않습니다. 회전, 수직 가속도, 골반 이동, 의도적 버팀, 핸들 지지, 비선형 벨트·등받이 접촉은 생략합니다.

**수치 처리**

- 1차 가속도 필터는 인과적이며, 정확한 지수 응답을 RK4 단계에서 샘플링합니다. 적분 하위 단계는 최대 5 ms입니다.
- `step`은 `0 < dt ≤ 50 ms`(최소 1 µs)를 요구합니다. 잘못되거나 유한하지 않은 입력, 더 긴 공백은 상태를 바꾸기 전에 예외를 냅니다. 원래 시각·품질 확인과 스트림 불연속 시 리셋은 호출자가 담당합니다.
- 모델의 각도·각속도 한계는 구동기 한계와 별개입니다. 잘리면 `limited=True`가 되며, 실제 사람의 가동 범위를 뜻하지 않습니다.
- 선택적 예측은 `q + horizon·q̇`(설정한 각도 범위로 제한)뿐이며 미래 샘플을 쓰지 않습니다. 종단 간 구동 지연을 측정하지 않았으므로 기본 `preview_horizon_s`는 0입니다.

## 5. 측정하지 않은 설계값

`nominal_design_config()`는 명시적인 **오프라인 설계 예시**입니다.

| 파라미터 | 값 | 근거 |
|---|---:|---|
| 기준 신장 / 질량 | 1.76 m / 75.3 kg | 인용 모델의 기준 체격 |
| 고유진동수 roll / sagittal | 1.2 / 1.0 Hz | 공학적 선택 (참가자 적합값 아님) |
| 감쇠비 | 0.85 | 〃 |
| 가속도 필터 시정수 | 0.10 s | 〃 |
| 각도 / 각속도 범위 | 20° / 90 °/s | 〃 |

이 값들은 실험 설정에 기록하고 민감도를 시험해야 합니다.

**민감도:** 같은 0.30 g 입력에서 roll 고유진동수를 0.6 / 1.2 / 2.0 Hz로 바꾸면 정적 예상 각도는 약 21.99° / 5.50° / 1.98°입니다(첫 값은 내부 20° 한계로 제한됨). 이는 참가자에 대한 신뢰구간이 아니라 식별되지 않은 변수의 영향을 보기 위한 설계 경우입니다. 진동수·감쇠를 식별하지 않고 개인별 정확도나 횡가속도 억제 효과를 주장하지 않습니다.

진동수·감쇠비를 고정하고 강성이 관성에 비례하면 **예상 각도에서 질량이 소거**됩니다. 의도된 동작이며 시험으로 확인합니다. 무거운 참가자를 입력한다고 볼스터 동작을 키울 근거가 되지 않습니다. 실제 강성을 독립적으로 식별하려면 추가 측정이 필요하며, 실험 결과에서 예상 몸통 각도와 실제 몸통 각도는 별도 필드로 유지해야 합니다.

## 6. 패드 면 정렬 (`alignment.py`)

**기준 인체 모델**로 신장 1.76 m, 체중 75 kg, 수직 기준 등받이 15°, 안전벨트 효과 제외, 상부 접촉 이동을 갖는 패드 면 정렬 목표를 정적 설정으로 사용했습니다. 런타임에 자세 측정은 들어오지 않습니다.

`alignment.py`는 스칼라 바깥쪽 기울기를 **등받이 국부 좌표의 기울기 대리값**으로 사용합니다. 설치된 CAD 좌표와 해부학 좌표가 일치한다는 뜻은 아닙니다. 15° 등받이 각도는 이미 인체 모델의 모멘트 팔 근사에 반영되어 있으므로, 추측한 CAD–시트 회전이나 좌우 반전 보정을 추가로 적용하지 않습니다.

```text
R   = Rz(−yaw)·Rx(pitch)
n   = (sin(yaw)·cos(pitch), cos(yaw)·cos(pitch), sin(pitch))   # 패드 면 법선 (국부)
n_t = (0, cos(θ), sin(θ))                                      # 가정한 몸통 측면 법선
pitch* = atan2(sin(θ), cos(yaw)·cos(θ))                        # 고정 Yaw에서 두 법선 사이 각 최소화
```

- 작동 볼스터 기준으로 `θ > 0`은 상체가 그 볼스터 쪽 바깥으로 기우는 것이며, 이때 정렬 Pitch는 양수(위쪽 가장자리가 상대적으로 바깥으로)입니다. 호출자가 오른쪽 양수 roll을 바깥쪽 양수 국부 규약으로 변환합니다.
- 이 해는 상·하 연결선의 바깥쪽 Y/Z **투영** 기울기를 `θ`와 같게 만들 뿐, 3D 선 전체가 몸통 축과 같아지지는 않습니다. Yaw가 0이 아니면 면 법선 불일치가 남습니다. `alignment_error_deg`는 이 가정상의 기하 잔차이며 측정된 접촉 오차가 아닙니다.
- 기구 결합 한계 때문에 이상적인 Pitch에 도달하지 못할 수 있으며, 최종 명령은 CAD·기구 한계를 독립적으로 유지합니다.

권장 기록 metadata:

```text
alignment_objective        = "closest_side_face_normal_at_fixed_yaw"
alignment_frame_assumption = "model_roll_as_local_backrest_tilt_proxy; installed_transform_unmeasured"
observed_body_pose_used    = false
```

설치된 좌표 변환과 실제 중립 접촉면을 측정하는 것이 이 모델을 개선하는 다음 단계입니다. 임의의 시상면 Pitch 보정으로 대신하는 것은 이 기하학으로 정당화되지 않습니다. 정렬 결과, 손의 핸들 지지, 동적 응답은 측정하지 않았으며 수치적 안정성과 문헌 기반 기하만으로 자세나 횡가속도가 상쇄된다고 주장할 수 없습니다.
