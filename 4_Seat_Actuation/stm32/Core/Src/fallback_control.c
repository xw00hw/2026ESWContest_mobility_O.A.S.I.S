/* Session-scoped latest-target tracking, opt-in only. Command pose is never
 * measured pose. Calibration identity must be checked by the dry-run-first
 * host; an explicit FB_ARM intentionally applies the recorded bench neutral. */
#ifndef OASIS_FALLBACK_CONTROL
#error "fallback_control requires OASIS_FALLBACK_CONTROL"
#endif
#include "stm32h7xx_hal.h"
#include "joint_control.h"
#include "joint_math.h"
#include "fallback_control.h"
#include "fallback_geometry.h"
#include "fallback_tuning.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static uint16_t pulses[4];
static int32_t pose[4], target[4];
static uint32_t session_floor, session, command_id, ping_counter;
static uint32_t last_alive, last_update, last_status;
static bool armed, moving, watchdog_ready;
static const char *fault="BOOT";
volatile FallbackDebugSnapshot g_fallback_debug;
static uint32_t debug_sequence;

static void debug_snapshot(void)
{
    /* The private sequence counter intentionally ignores debugger writes. */
    g_fallback_debug.sequence=++debug_sequence;
    __DMB();
    g_fallback_debug.schema_version=1U;
    g_fallback_debug.mcu_tick_ms=HAL_GetTick();
    g_fallback_debug.armed=armed?1U:0U;
    g_fallback_debug.moving=moving?1U:0U;
    g_fallback_debug.session_id=session;
    g_fallback_debug.command_id=command_id;
    for (unsigned i=0;i<4;i++) {
        g_fallback_debug.commanded_pose_mdeg[i]=pose[i];
        g_fallback_debug.target_pose_mdeg[i]=target[i];
        g_fallback_debug.commanded_pulse_us[i]=pulses[i];
    }
    g_fallback_debug.lease_elapsed_ms=armed?(uint32_t)(HAL_GetTick()-last_alive):0U;
    unsigned i=0;
    while (i<sizeof g_fallback_debug.fault-1U && fault[i]) {
        g_fallback_debug.fault[i]=fault[i]; ++i;
    }
    while (i<sizeof g_fallback_debug.fault) g_fallback_debug.fault[i++]=0;
    const FallbackTuneValues *tune=Fallback_TuneActive();
    g_fallback_debug.tuning_revision=Fallback_TuneRevision();
    g_fallback_debug.tuning_handled_seq=Fallback_TuneHandled();
    g_fallback_debug.tuning_result_code=Fallback_TuneResult();
    g_fallback_debug.tuning_active[0]=tune->yaw_max_mdeg;
    g_fallback_debug.tuning_active[1]=tune->pitch_max_mdeg;
    g_fallback_debug.tuning_active[2]=tune->yaw_rate_mdeg_s;
    g_fallback_debug.tuning_active[3]=tune->pitch_rate_mdeg_s;
    g_fallback_debug.tuning_active[4]=tune->full_scale_ay_mmps2;
    g_fallback_debug.tuning_active[5]=tune->pitch_scale_permille;
    __DMB();
    g_fallback_debug.sequence=++debug_sequence;
}

void Control_Fault(const char *reason)
{
    Fallback_MotorOff();
    armed=moving=false; session=0;
    memset(pulses,0,sizeof pulses);
    memcpy(target,pose,sizeof target);
    fault=reason;
    debug_snapshot();
}

void Fallback_SafetyPoll(void)
{
    if (armed && (uint32_t)(HAL_GetTick()-last_alive)>=FALLBACK_LEASE_MS)
        Control_Fault("LEASE");
}

void Control_Heartbeat(void) { /* Pressure PING cannot renew motor authority. */ }

static void state(char *line, size_t size)
{
    snprintf(line,size,"#STATE,%" PRIu32 ",%u,%u,%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
        ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%u,%u,%u,%u,%" PRIu32 ",%s\r\n",
        HAL_GetTick(),armed?1U:0U,moving?1U:0U,pose[0],pose[1],pose[2],pose[3],
        target[0],target[1],target[2],target[3],pulses[0],pulses[1],pulses[2],pulses[3],command_id,fault);
}

/* Strict unsigned decimal, overflow checked before multiply. No +/-/space. */
static bool uint_field(const char **cursor,uint32_t *value,char delimiter)
{
    const char *s=*cursor;
    if (*s<'0' || *s>'9') return false;
    uint32_t n=0;
    do {
        unsigned digit=(unsigned)(*s++-'0');
        if (n>UINT32_MAX/10U || (n==UINT32_MAX/10U && digit>UINT32_MAX%10U)) return false;
        n=n*10U+digit;
    } while (*s>='0' && *s<='9');
    if (*s!=delimiter) return false;
    *value=n; *cursor=delimiter?s+1:s;
    return true;
}

static bool angle_field(const char **cursor,int32_t *value,bool pitch,char delimiter)
{
    bool negative=**cursor=='-';
    if (negative) ++*cursor;
    uint32_t n;
    if (!uint_field(cursor,&n,delimiter) || n>(uint32_t)(pitch?JOINT_PITCH_LIMIT_MDEG:JOINT_YAW_LIMIT_MDEG) ||
        (negative && !pitch)) return false;
    *value=negative?-(int32_t)n:(int32_t)n;
    return true;
}

static void reject(uint32_t id,const char *reason)
{
    if (armed) Control_Fault("PROTOCOL");
    char line[96];
    snprintf(line,sizeof line,"#ERR,%" PRIu32 ",%s\r\n",id,reason);
    FSR_Transmit(line);
}

static const char *round_cad_candidate(int32_t next_pose[4], uint16_t next[4])
{
    if (Fallback_CadPoseAllowed(next_pose)) return NULL;
    if (!Fallback_CadRoundInward(next_pose)) return "CAD_PATH";
    /* Moving a boundary waypoint inward by 1mdeg can cross an integer PWM
     * rounding boundary. Retain the independent 2us guard: shorten this one
     * waypoint toward the current command and retry, never widen the limits.
     * Sixteen halvings cover the entire legacy 27k/16k mdeg coordinate range. */
    for (unsigned attempt=0;attempt<16;attempt++) {
        if (!Joint_PosePulses(next_pose,next)) return "PATH_BOUNDS";
        bool rate_ok=Fallback_TuneStepAllowed(pose,next_pose);
        for (unsigned i=0;i<4;i++) {
            int delta=(int)next[i]-(int)pulses[i];
            if (delta>2 || delta< -2) rate_ok=false;
        }
        if (rate_ok) return NULL;
        for (unsigned i=0;i<4;i++) next_pose[i]=pose[i]+(next_pose[i]-pose[i])/2;
        if (!Fallback_CadRoundInward(next_pose)) return "CAD_PATH";
    }
    return "RATE_BOUNDS";
}

static int command_body(const char *command)
{
    Fallback_SafetyPoll(); /* An expired command cannot revive a lease. */
    Fallback_TunePoll(armed);
    char line[384];
    if (!strcmp(command,"CONTROL_HELLO")) {
        const uint16_t neutral[4]=JOINT_BOOT_PULSES;
        snprintf(line,sizeof line,"#HELLO,OASIS_FALLBACK,v2,yaw_min_mdeg=0,yaw_limit_mdeg=27000,pitch_limit_mdeg=8000,servo_range_deg=%u,neutral_us=%u:%u:%u:%u,boot_pwm=OFF,lease_ms=500,tick_ms=20,max_step_us=2,ds_outputs=4,ik_revision=8,session_floor=%" PRIu32 "\r\n",
            (unsigned)JOINT_SERVO_RANGE_DEG,(unsigned)neutral[0],(unsigned)neutral[1],
            (unsigned)neutral[2],(unsigned)neutral[3],session_floor);
        FSR_Transmit(line); return 1;
    }
    if (!strcmp(command,"CAD_INFO")) { FSR_Transmit(Fallback_CadDescriptor()); return 1; }
    if (!strcmp(command,"TUNE_STATE")) { Fallback_TuneDescribe(line,sizeof line); FSR_Transmit(line); return 1; }
    if (!strcmp(command,"STATE")) { state(line,sizeof line); FSR_Transmit(line); return 1; }
    if (!strcmp(command,"FB_OFF")) {
        const char *reason=(!strcmp(fault,"NONE") || !strcmp(fault,"BOOT"))?"OFF":fault;
        Control_Fault(reason);
        if (!Fallback_PwmValid(pulses,false)) { fault="HARDWARE"; reject(0,"HARDWARE"); return 1; }
        FSR_Transmit("#ACK,FB_OFF\r\n"); return 1;
    }
    if (!strncmp(command,"FB_ARM,",7)) {
        const char *cursor=command+7; uint32_t sid=0,tune_revision=0;
        if (!uint_field(&cursor,&sid,',') || !uint_field(&cursor,&tune_revision,'\0') || !sid) { reject(0,"INVALID"); return 1; }
        if (armed) { reject(0,"ALREADY_ARMED"); return 1; }
        if (tune_revision!=Fallback_TuneRevision()) { reject(0,"TUNE_REVISION"); return 1; }
        if (sid<=session_floor) { reject(0,"SESSION_REPLAY"); return 1; }
        if (!watchdog_ready) { reject(0,"WATCHDOG"); return 1; }
        session_floor=sid; /* Consume before enabling; failed ARM cannot be replayed. */
        const uint16_t neutral[4]=JOINT_BOOT_PULSES;
        if (!Fallback_PwmStart(neutral)) { Control_Fault("HARDWARE"); reject(0,"HARDWARE"); return 1; }
        memset(pose,0,sizeof pose); memset(target,0,sizeof target);
        memcpy(pulses,neutral,sizeof pulses);
        command_id=ping_counter=0; session=sid; armed=true; moving=false; fault="NONE";
        last_alive=last_update=HAL_GetTick();
        snprintf(line,sizeof line,"#ACK,FB_ARM,%" PRIu32 ",%" PRIu32 "\r\n",sid,tune_revision);
        FSR_Transmit(line); return 1;
    }
    if (!strncmp(command,"FB_PING,",8)) {
        const char *cursor=command+8; uint32_t sid=0,counter=0;
        if (!uint_field(&cursor,&sid,',') || !uint_field(&cursor,&counter,'\0') || !sid || !counter)
            { reject(0,"INVALID"); return 1; }
        if (!armed) { reject(0,"DISARMED"); return 1; }
        if (sid!=session) { reject(0,"SESSION"); return 1; }
        if (counter<=ping_counter) { reject(0,"PING_REPLAY"); return 1; }
        ping_counter=counter; last_alive=HAL_GetTick();
        snprintf(line,sizeof line,"#ACK,FB_PING,%" PRIu32 ",%" PRIu32 "\r\n",sid,counter);
        FSR_Transmit(line); return 1;
    }
    if (!strncmp(command,"FB_SET,",7)) {
        const char *cursor=command+7; uint32_t sid=0,id=0; int32_t requested[4]; uint16_t checked[4];
        bool valid=uint_field(&cursor,&sid,',') && uint_field(&cursor,&id,',') && sid && id;
        for (unsigned i=0;i<4 && valid;i++) valid=angle_field(&cursor,&requested[i],(i&1U)!=0,i==3?'\0':',');
        if (!valid) { reject(id,"INVALID"); return 1; }
        if (!armed) { reject(id,"DISARMED"); return 1; }
        if (sid!=session) { reject(id,"SESSION"); return 1; }
        if (id<=command_id) { reject(id,"SET_REPLAY"); return 1; }
        if (!Fallback_CadPoseAllowed(requested)) {
            Control_Fault("CAD_BOUNDS"); reject(id,"CAD_BOUNDS"); return 1;
        }
        if (!Fallback_TuneTargetAllowed(requested)) {
            Control_Fault("TUNE_BOUNDS"); reject(id,"TUNE_BOUNDS"); return 1;
        }
        if (!Joint_PosePulses(requested,checked)) { reject(id,"MOTOR_BOUNDS"); return 1; }
        Fallback_SafetyPoll();
        if (!armed) { reject(id,"DISARMED"); return 1; }
        /* This changes only the latest target. Never reset the motion clock. */
        memcpy(target,requested,sizeof target); command_id=id; last_alive=HAL_GetTick();
        moving=memcmp(pose,target,sizeof pose)!=0;
        snprintf(line,sizeof line,"#ACK,FB_SET,%" PRIu32 ",%" PRIu32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "\r\n",
                 sid,id,target[0],target[1],target[2],target[3]);
        FSR_Transmit(line); return 1;
    }
    if (!strncmp(command,"FB_",3) || !strcmp(command,"ARM") || !strcmp(command,"HOLD") ||
        !strcmp(command,"SET") || !strncmp(command,"SET,",4)) {
        reject(0,"UNSUPPORTED"); return 1;
    }
    return 0;
}

int Control_Command(const char *command)
{
    int handled=command_body(command);
    debug_snapshot();
    return handled;
}

void Control_Init(void)
{
    Fallback_MotorOff();
    memset(pulses,0,sizeof pulses); memset(pose,0,sizeof pose); memset(target,0,sizeof target);
    session_floor=session=command_id=ping_counter=0;
    armed=moving=false; fault="BOOT";
    Fallback_TuneInit();
    last_alive=last_update=last_status=HAL_GetTick();
    watchdog_ready=Fallback_WatchdogStart();
    if (!watchdog_ready) Control_Fault("WATCHDOG");
    debug_snapshot();
}

void Control_Poll(void)
{
    Fallback_SafetyPoll();
    Fallback_TunePoll(armed);
    /* Check preloads before a due write can hide corruption. */
    if (!Fallback_PwmValid(pulses,armed)) Control_Fault("HARDWARE");
    uint32_t now=HAL_GetTick();
    if (armed && moving && (uint32_t)(now-last_update)>=FALLBACK_TICK_MS && Fallback_PwmFrameReady()) {
        int32_t next_pose[4]; uint16_t next[4];
        const char *candidate_fault=NULL;
        memcpy(next_pose,target,sizeof next_pose);
        Fallback_TuneLimitStep(pose,next_pose);
        if (!Fallback_CadPoseAllowed(pose) || !Fallback_CadPoseAllowed(target)) Control_Fault("CAD_BOUNDS");
        else if (!Fallback_TuneTargetAllowed(pose) || !Fallback_TuneTargetAllowed(target)) Control_Fault("TUNE_BOUNDS");
        else if (!Joint_PoseStep(pose,next_pose,FALLBACK_STEP_US,next)) Control_Fault("PATH_BOUNDS");
        else if ((candidate_fault=round_cad_candidate(next_pose,next))!=NULL)
            Control_Fault(candidate_fault);
        else {
            bool valid=Fallback_TuneStepAllowed(pose,next_pose);
            for (unsigned i=0;i<4;i++) {
                int delta=(int)next[i]-(int)pulses[i];
                if (next[i]<500 || next[i]>2500 || delta>2 || delta< -2) valid=false;
            }
            Fallback_SafetyPoll(); /* Do not commit after a long computation expires authority. */
            if (armed && !Fallback_PwmValid(pulses,true)) Control_Fault("HARDWARE");
            else if (armed && !valid) Control_Fault("RATE_BOUNDS");
            else if (armed && Fallback_PwmStage(next)) {
                memcpy(pose,next_pose,sizeof pose); memcpy(pulses,next,sizeof pulses);
                last_update=HAL_GetTick();
                moving=memcmp(pose,target,sizeof pose)!=0;
            }
        }
    }
    now=HAL_GetTick();
    if ((uint32_t)(now-last_status)>=200U) {
        char line[224]; state(line,sizeof line);
        if (FSR_TryTelemetry(line)) last_status=now;
    }
    debug_snapshot();
}

void Fallback_EndLoop(void)
{
    Fallback_SafetyPoll();
    if (watchdog_ready) Fallback_WatchdogRefresh();
}
