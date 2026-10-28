#ifndef JOINT_MATH_H
#define JOINT_MATH_H
#include <stdint.h>
#define JOINT_YAW_LIMIT_MDEG 27000
#define JOINT_PITCH_LIMIT_MDEG 8000
#ifdef OASIS_FALLBACK_CONTROL
/* Corrected mechanical zero, UR/UL/DR/DL (D3/D4/D5/D6).
 * This correction belongs to the current fallback/RL installation only. */
#define JOINT_BOOT_PULSES {1800,1800,1200,1200}
#else
/* Preserve the recorded acquisition-v6 calibration and its data lineage. */
#define JOINT_BOOT_PULSES {1800,1200,1200,1800}
#endif
#define MOTOR_RATE_MDEG_S 15000
#define CONTROL_TIMEOUT_MS 1500U
#ifdef OASIS_FALLBACK_CONTROL
/* Experiment A: the four DS5180 motors are configured for 270 degrees.
 * The mechanical-zero calibration is selected separately above. */
#define JOINT_SERVO_RANGE_DEG 270U
#else
#define JOINT_SERVO_RANGE_DEG 270U
#endif
/* Exact C4_R8 IK. D3=-theta1, D4=theta1, D5=theta2, D6=-theta2.
   Yaw <=27deg; Pitch +/-8deg; each pose also passes exact IK. Mirrored 1200us zero. */
int Joint_CrankAngles(double yaw_deg, double pitch_deg, double phi[2]);
int Joint_Pulses(int32_t yaw, int32_t pitch, uint16_t pulse[4]);
uint32_t Joint_Duration(int32_t yaw0, int32_t pitch0, int32_t yaw1, int32_t pitch1);
int Joint_LimitStep(int32_t yaw, int32_t pitch, int32_t *next_yaw, int32_t *next_pitch, unsigned limit_us, uint16_t pulse[4]);
/* pose = left yaw,pitch (D4,D6), right yaw,pitch (D3,D5). */
int Joint_PosePulses(const int32_t pose[4], uint16_t pulse[4]);
int Joint_PoseStep(const int32_t pose[4], int32_t next[4], unsigned limit_us, uint16_t pulse[4]);
#endif
