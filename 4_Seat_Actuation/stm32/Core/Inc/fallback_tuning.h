#ifndef FALLBACK_TUNING_H
#define FALLBACK_TUNING_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t yaw_max_mdeg;
    int32_t pitch_max_mdeg;
    int32_t yaw_rate_mdeg_s;
    int32_t pitch_rate_mdeg_s;
    int32_t full_scale_ay_mmps2;
    int32_t pitch_scale_permille;
} FallbackTuneValues;

/* CubeIDE Live Expressions: edit the six integer values first, then increase
 * request_seq LAST. Commit is accepted only while PWM is OFF. This mailbox
 * cannot ARM, renew a lease, command a pose or alter the fixed CAD profile. */
typedef struct {
    FallbackTuneValues values;
    uint32_t request_seq;
} FallbackTuneRequest;
extern volatile FallbackTuneRequest g_experiment_a_tune;

enum { FALLBACK_TUNE_READY=0, FALLBACK_TUNE_APPLIED=1, FALLBACK_TUNE_BUSY=2,
       FALLBACK_TUNE_RANGE=3, FALLBACK_TUNE_SEQUENCE=4 };
void Fallback_TuneInit(void);
void Fallback_TunePoll(bool armed);
uint32_t Fallback_TuneRevision(void);
uint32_t Fallback_TuneHandled(void);
uint32_t Fallback_TuneResult(void);
const FallbackTuneValues *Fallback_TuneActive(void);
void Fallback_TuneDescribe(char *line, size_t size);
bool Fallback_TuneTargetAllowed(const int32_t pose[4]);
void Fallback_TuneLimitStep(const int32_t pose[4], int32_t next[4]);
bool Fallback_TuneStepAllowed(const int32_t pose[4], const int32_t next[4]);
#endif
