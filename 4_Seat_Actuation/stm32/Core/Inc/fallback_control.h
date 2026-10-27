#ifndef FALLBACK_CONTROL_H
#define FALLBACK_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#define FALLBACK_LEASE_MS 500U
#define FALLBACK_TICK_MS 20U
#define FALLBACK_STEP_US 2U
/* Observation-only debugger copy. Firmware never reads this memory as a
 * command, tuning source, ARM request or lease renewal. Angles/PWM are command
 * values; no shaft position/current/torque sensor is present. Read sequence
 * before and after a snapshot: equal even values identify a coherent copy. */
typedef struct {
    uint32_t sequence;
    uint32_t schema_version;
    uint32_t mcu_tick_ms;
    uint32_t armed;
    uint32_t moving;
    uint32_t session_id;
    uint32_t command_id;
    int32_t commanded_pose_mdeg[4]; /* LY, LP, RY, RP */
    int32_t target_pose_mdeg[4];
    uint32_t commanded_pulse_us[4]; /* D3/UR, D4/UL, D5/DR, D6/DL */
    uint32_t lease_elapsed_ms;
    char fault[32];
    uint32_t tuning_revision;
    uint32_t tuning_handled_seq;
    uint32_t tuning_result_code; /* 0 READY, 1 APPLIED, 2 BUSY, 3 RANGE, 4 SEQUENCE */
    int32_t tuning_active[6]; /* See g_experiment_a_tune.values field order. */
} FallbackDebugSnapshot;
extern volatile FallbackDebugSnapshot g_fallback_debug;
/* May be called before HAL/clock setup. Only PE9/11/13/14 and TIM1 change. */
void Fallback_MotorOff(void);
/* Deadline check only, never renews authority or advances a target. */
void Fallback_SafetyPoll(void);
/* Feed only after one complete FSR/control main-loop iteration. */
void Fallback_EndLoop(void);
bool Fallback_PwmStart(const uint16_t pulse[4]);
bool Fallback_PwmValid(const uint16_t pulse[4], bool armed);
bool Fallback_PwmFrameReady(void);
bool Fallback_PwmStage(const uint16_t pulse[4]);
bool Fallback_WatchdogStart(void);
void Fallback_WatchdogRefresh(void);
#endif
