#ifndef OASIS_FALLBACK_CONTROL
#error "fallback_tuning is exclusive to the Experiment A fallback image"
#endif
#include "stm32h7xx_hal.h"
#include "fallback_control.h"
#include "fallback_geometry_profile.h"
#include "fallback_tuning.h"
#include <inttypes.h>
#include <stdio.h>

#define DEFAULT_VALUES {15000,6000,2000,2000,2942,1000}
volatile FallbackTuneRequest g_experiment_a_tune={DEFAULT_VALUES,0};
static FallbackTuneValues active=DEFAULT_VALUES;
static uint32_t revision, handled, observed, sequence_floor, result;

void Fallback_TuneInit(void)
{
    const FallbackTuneValues defaults=DEFAULT_VALUES;
    active=defaults;
    g_experiment_a_tune.values=defaults;
    g_experiment_a_tune.request_seq=0;
    revision=handled=observed=sequence_floor=0;
    result=FALLBACK_TUNE_READY;
}

static bool valid(const FallbackTuneValues *value)
{
    return value->yaw_max_mdeg>=100 && value->yaw_max_mdeg<=FALLBACK_CAD_YAW_MAX_MDEG &&
        value->pitch_max_mdeg>=0 && value->pitch_max_mdeg<=FALLBACK_CAD_PITCH_MAX_MDEG &&
        value->yaw_rate_mdeg_s>=100 && value->yaw_rate_mdeg_s<=20000 &&
        value->pitch_rate_mdeg_s>=100 && value->pitch_rate_mdeg_s<=20000 &&
        value->full_scale_ay_mmps2>=500 && value->full_scale_ay_mmps2<=8000 &&
        value->pitch_scale_permille>=10 && value->pitch_scale_permille<=1000;
}

void Fallback_TunePoll(bool armed)
{
    uint32_t seq=g_experiment_a_tune.request_seq;
    if (seq==observed) return;
    __DMB();
    FallbackTuneValues candidate=g_experiment_a_tune.values;
    __DMB();
    if (seq!=g_experiment_a_tune.request_seq) return;
    observed=handled=seq;
    if (!seq || seq<=sequence_floor) { result=FALLBACK_TUNE_SEQUENCE; return; }
    sequence_floor=seq; /* Failed commits consume the sequence too. */
    if (armed) { result=FALLBACK_TUNE_BUSY; return; }
    if (!valid(&candidate)) { result=FALLBACK_TUNE_RANGE; return; }
    active=candidate;
    revision=seq;
    result=FALLBACK_TUNE_APPLIED;
}

uint32_t Fallback_TuneRevision(void) { return revision; }
uint32_t Fallback_TuneHandled(void) { return handled; }
uint32_t Fallback_TuneResult(void) { return result; }
const FallbackTuneValues *Fallback_TuneActive(void) { return &active; }

void Fallback_TuneDescribe(char *line, size_t size)
{
    snprintf(line,size,"#TUNE,OASIS_A,v1,revision=%" PRIu32
        ",yaw_max_mdeg=%" PRId32 ",pitch_max_mdeg=%" PRId32
        ",yaw_rate_mdeg_s=%" PRId32 ",pitch_rate_mdeg_s=%" PRId32
        ",full_scale_ay_mmps2=%" PRId32 ",pitch_scale_permille=%" PRId32 "\r\n",
        revision,active.yaw_max_mdeg,active.pitch_max_mdeg,
        active.yaw_rate_mdeg_s,active.pitch_rate_mdeg_s,
        active.full_scale_ay_mmps2,active.pitch_scale_permille);
}

bool Fallback_TuneTargetAllowed(const int32_t pose[4])
{
    if (!pose) return false;
    for (unsigned side=0;side<2;side++) {
        int32_t yaw=pose[2U*side],pitch=pose[2U*side+1U];
        if (yaw<0 || yaw>active.yaw_max_mdeg || pitch< -active.pitch_max_mdeg ||
            pitch>active.pitch_max_mdeg) return false;
    }
    return true;
}

static int32_t step_limit(unsigned axis)
{
    int32_t rate=(axis&1U)?active.pitch_rate_mdeg_s:active.yaw_rate_mdeg_s;
    return (int32_t)((int64_t)rate*FALLBACK_TICK_MS/1000);
}

void Fallback_TuneLimitStep(const int32_t pose[4], int32_t next[4])
{
    /* One common rational fraction preserves a straight coupled-pose path. */
    int64_t numerator=1,denominator=1;
    for (unsigned i=0;i<4;i++) {
        int64_t delta=(int64_t)next[i]-pose[i];
        if (delta<0) delta= -delta;
        int32_t limit=step_limit(i);
        if (delta && (int64_t)limit*denominator<delta*numerator) {
            numerator=limit; denominator=delta;
        }
    }
    for (unsigned i=0;i<4;i++)
        next[i]=pose[i]+(int32_t)(((int64_t)next[i]-pose[i])*numerator/denominator);
}

bool Fallback_TuneStepAllowed(const int32_t pose[4], const int32_t next[4])
{
    for (unsigned i=0;i<4;i++) {
        int64_t delta=(int64_t)next[i]-pose[i];
        if (delta<0) delta= -delta;
        if (delta>step_limit(i)) return false;
    }
    return true;
}
