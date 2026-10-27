# Experiment A: meaning of the torso model

This module predicts an **uncontrolled, hypothetical torso response to simulated vehicle acceleration**. It does not observe the seated person, predict the closed-loop result after assistance, or estimate a contact force. The physical rig is stationary; BeamNG acceleration is not an acceleration measured on its occupant. Precision of numerical integration does not establish physiological prediction accuracy.

## Primary evidence and its limits

1. [Desai et al., 2023, *Computationally efficient human body modelling for real time motion comfort assessment*](https://arxiv.org/pdf/2306.12279), sections 2–2.1 and Table 2. Their multibody model has a reference stature of 1.76 m and mass of 75.3 kg. It distinguishes anthropometric inertial parameters from stiffness/damping parameters that require identification. Its validated model includes multiple joints, contacts and experimentally fitted parameters; our two-axis reduction does **not** inherit that validation. Exact source excerpt: “There can be significant individual variation in muscle activation patterns and joint stiffness”.
2. [Vette et al., 2014, *Multidirectional quantification of trunk stiffness and damping during unloaded natural sitting*](https://pubmed.ncbi.nlm.nih.gov/24246838/), DOI 10.1016/j.medengphy.2013.10.005. Fifteen healthy seated participants were perturbed in eight directions. A mass–spring–damper description was used; lateral and anterior behavior differed, with approximately symmetric left/right estimates. This supports separate directional dynamics, not a universal stiffness or damping value for our seat. Exact source excerpt: “both quantities are smallest in the anterior and largest in the lateral directions”.
3. [Mirakhorlo et al., 2022, *Effects of seat back height and posture on 3D vibration transmission to pelvis, trunk and head*](https://arxiv.org/html/2207.01989v1). The experiment varies seat support and posture under multidirectional vibration. It shows why neutral posture, support geometry and task need recording before comparison. Exact source excerpt: “Seat back support height and sitting posture substantially affected vibration transmission”.
4. [de Leva, 1996, *Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters*](https://pubmed.ncbi.nlm.nih.gov/8872282/). The study explains the need for consistent segment landmarks when using anthropometric mass, center-of-mass and gyration parameters. Its abstract was inspected, but full tables were not independently retrieved in this task; numerical defaults are therefore traced to the directly inspected Desai table instead of mixing different segment definitions.

Retrieved 2026-10-26. The sources above are primary studies. The reductions below are our engineering assumptions, not equations or calibrated values claimed from those papers.

## Reference anthropometry reduction

The five head/neck/torso entries in Desai Table 2 are collapsed about the table's pelvis origin. Arms are described as incorporated into the upper torso. Table coordinates are in cm, converted below to m. The resulting model is a lumped approximation, not a recreation of the authors' multibody model. No sex-specific or population-average accuracy is claimed.

| Segment | Mass kg | Ixx about COM kg·m² | Iyy about COM kg·m² | x m | z m |
|---|---:|---:|---:|---:|---:|
| Head | 6.23 | .031 | .031 | -.05461 | .69104 |
| Neck | 1.60 | .003 | .004 | -.09420 | .57393 |
| Upper torso | 8.93 | .238 | .146 | -.08231 | .34692 |
| Middle torso | 7.70 | .238 | .146 | -.03827 | .15927 |
| Lower torso | 10.70 | .137 | .078 | .002164 | .05172 |

All table y coordinates are zero. Applying the parallel-axis rule gives:

- `m_ref = sum(m_i) = 35.16 kg`.
- `h_ref = sum(m_i*z_i)/m_ref = 0.28729368031854385 m`.
- `Iroll_ref = sum(Ixx_i + m_i*z_i²) = 5.447788137970001 kg m²`.
- `Ipitch_ref = sum(Iyy_i + m_i*(x_i²+z_i²)) = 5.310393044243201 kg m²`.

For configured whole-body mass `M` and stature `H`, default geometric similarity uses `m=M*(35.16/75.3)`, `h=H*(h_ref/1.76)`, and `I=m*H²*reference_factor`. These proportions can be overridden. This scaling assumes geometrically similar bodies and does not determine personal mass distribution, bracing, belt effects, contact compliance, or muscle response.

Seat recline is backward from vertical. Only the moment-arm approximation `h_vertical=h*cos(recline)` is applied. The effective modal inertias are not reidentified for the reclined posture. Large changes in seat posture require new model identification; the parameter is not a full 3D anthropometric transformation.

## State, signs, equations and timing

- Input: gravity-excluded BeamNG `ax, ay` in m/s², vehicle `+x=forward, +y=left`. No pressure, camera, recorded future samples or actuator response enters this module.
- State angles are deviations from an assumed neutral torso. `roll>0` means occupant right lean; `pitch>0` means forward lean.
- Positive `ay` drives right lean. Positive `ax` drives backward lean; braking drives forward lean.
- The R8 pad's mechanical **Pitch** changes upper/lower lateral protrusion. It is not the anatomical sagittal pitch state. Any actuator mapping must use the pad geometry and the selected support policy separately.

For each axis, `I*q_ddot + C*q_dot + K_net*q = m*h_vertical*a`, with `a=ay` for roll and `a=-ax` for sagittal pitch. `omega=2*pi*f`, `K_net=I*omega²`, `C=2*zeta*I*omega`. `K_net` is the positive **net** restoring coefficient around neutral; gravity and baseline postural support are already implicit. Do not subtract or add `m*g*h` again. These are independent small-angle modes; rotation, vertical acceleration, moving pelvis, voluntary bracing, steering-wheel support and nonlinear belt/backrest contact are omitted.

The first-order acceleration filter is causal. Its exact exponential response is sampled at RK4 stages, with integration substeps at most 5 ms. `step` requires `0<dt<=50 ms` (minimum supported 1 μs); invalid/nonfinite input or a longer gap raises before state mutation. The caller checks original timestamps/quality and resets on stream discontinuity. Physical model angle/rate bounds are distinct from actuator angle/rate limits; clipping sets `limited=True` and does not establish real human range of motion.

Optional preview is only `q + horizon*q_dot`, bounded to the configured angle domain. It uses current internal state and no future samples. Default horizon is zero because end-to-end actuator delay has not been measured. A nonzero horizon is not an experimentally verified forecast.

## Choices that remain unmeasured

`nominal_design_config()` is explicitly an **offline design example**. Reference stature/mass comes from the cited model. Frequencies **1.2 Hz roll / 1.0 Hz sagittal**, damping ratios **0.85**, filter **0.10 s**, angle domain **20°** and rate domain **90°/s** are engineering choices, not physiological data fitted to this participant. They must be logged in the experiment configuration and tested for sensitivity.

Mass cancels from predicted angle when frequency/damping remain fixed and stiffness scales with inertia. This is intentional and tested; merely entering a heavier participant does not justify increasing bolster motion. Identifying physical stiffness independently would require additional measurements. Expected torso angle and achieved torso angle must remain separate fields in experiment results.

Participant height/mass, neutral seat angle, belt treatment and support objective have now been supplied as recorded below. Pitch/rates are engineering proposals in the candidate configuration. Installed alignment, hands-on-wheel support and dynamic response remain unmeasured. Stable numerical behavior and literature-derived geometry alone cannot support a claim that posture or lateral acceleration has been cancelled.

## Conforming pad-face orientation: explicit frame assumption

The participant later specified **1.76 m, 75 kg, 15° backward seat recline**, with belt effects omitted, and selected a conforming pad-face objective with upper contact motion. These are static experiment configuration inputs; no body-pose measurement enters runtime. The anthropometric reference above remains the source reference, distinct from this participant configuration.

`alignment.py` uses the scalar predicted outward torso tilt as a **local backrest-frame proxy**. It does not establish that the installed CAD frame and anatomical frame coincide. The 15° recline is already present in the human model's approximate moment arm. No additional guessed CAD-to-seat rotation or mirrored-side recline correction is applied.

For `R=Rz(-yaw)*Rx(pitch)`, the local pad face normal is
`n=(sin(yaw)*cos(pitch), cos(yaw)*cos(pitch), sin(pitch))`.
The assumed torso-side face normal is `n_t=(0,cos(theta),sin(theta))`.
At the Yaw selected for closure, the Pitch minimizing the angle between these normals is
`pitch=atan2(sin(theta), cos(yaw)*cos(theta))`.
For the active bolster, `theta>0` means the upper torso leans outward toward that bolster; conforming Pitch is positive, moving the upper edge relatively outward. The caller converts signed right-positive body roll into this outward-positive local convention.

This solution also makes the upper/lower line's outward Y/Z **projected** tilt equal to `theta`. It does not make the full 3D line equal to the torso axis, and nonzero Yaw generally leaves a full face-normal mismatch. `alignment_error_deg` reports that assumed geometric residual, not measured contact error. Combined mechanism limits can prevent reaching the ideal Pitch; the constrained command must retain those independent limits.

Recommended run metadata: `alignment_objective="closest_side_face_normal_at_fixed_yaw"`, `alignment_frame_assumption="model_roll_as_local_backrest_tilt_proxy; installed_transform_unmeasured"`, and `observed_body_pose_used=false`. Measuring the installed coordinate transform and actual neutral contact plane is the next way to improve this model; substituting an arbitrary sagittal-body Pitch correction is not justified by this geometry.
