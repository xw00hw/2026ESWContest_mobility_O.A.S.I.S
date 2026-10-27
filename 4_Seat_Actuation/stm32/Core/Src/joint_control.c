#include "stm32h7xx_hal.h"
#include "fsr_board_pins.h"
#include "joint_control.h"
#include "joint_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

static TIM_HandleTypeDef timer;
static uint16_t pulses[4] = JOINT_BOOT_PULSES;
static const uint32_t channels[4] = {TIM_CHANNEL_3,TIM_CHANNEL_4,TIM_CHANNEL_2,TIM_CHANNEL_1};
static int32_t pose[4], from[4], target[4];
static uint32_t begun, duration, last_update, last_alive, last_status, command_id;
static unsigned armed, moving;
static const char *fault = "BOOT";

static void update(uint32_t now)
{
    if (!moving || (uint32_t)(now-last_update) < 20U) return;
    uint32_t dt=now-last_update;
    if(dt>40U) dt=40U;
    last_update = now;
    uint32_t elapsed = now-begun;
    float t = elapsed >= duration ? 1.0f : (float)elapsed / (float)duration;
    float f = t*t*t*(10.0f+t*(-15.0f+6.0f*t));
    int32_t next_pose[4];
    for(unsigned i=0;i<4;i++) {
        float value=(float)from[i]+(float)(target[i]-from[i])*f;
        next_pose[i]=elapsed >= duration ? target[i] : (int32_t)(value+(value>=0 ? 0.5f : -0.5f));
    }
    uint16_t next[4];
    unsigned limit=(unsigned)((uint64_t)MOTOR_RATE_MDEG_S*2000U*dt/270000000U);
    if(!Joint_PoseStep(pose,next_pose,limit,next)) {
        Control_Fault("INTERNAL_BOUNDS"); return;
    }
    memcpy(pose,next_pose,sizeof pose);
    memcpy(pulses,next,sizeof pulses);
    for (unsigned i=0; i<4; i++) __HAL_TIM_SET_COMPARE(&timer,channels[i],pulses[i]);
    if (elapsed >= duration && !memcmp(pose,target,sizeof pose)) moving = 0U;
}

void Control_Fault(const char *reason)
{
    /* Hold the last valid PWM commands, latch disarmed; never auto-resume.
       Does not measure position or remove torque/external motor power. */
    moving = armed = 0U;
    memcpy(target,pose,sizeof pose);
    fault = reason;
}

void Control_Heartbeat(void) { last_alive = HAL_GetTick(); }

static void status(char *line, unsigned size)
{
    snprintf(line,size,"#STATE,%" PRIu32 ",%u,%u,%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
        ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32
        ",%u,%u,%u,%u,%" PRIu32 ",%s\r\n",HAL_GetTick(),armed,moving,
        pose[0],pose[1],pose[2],pose[3],target[0],target[1],target[2],target[3],
        pulses[0],pulses[1],pulses[2],pulses[3],command_id,fault);
}

/* Strict decimal parser: no whitespace, exponent, empty field or overflow. */
static int number(const char **cursor, int32_t minimum, int32_t maximum, int32_t *value, char delimiter)
{
    const char *s = *cursor;
    if (*s == '-') s++;
    if (*s < '0' || *s > '9') return 0;
    while (*s >= '0' && *s <= '9') s++;
    if (*s != delimiter) return 0;
    errno = 0;
    char *end;
    long parsed = strtol(*cursor,&end,10);
    if (errno || end != s || parsed < minimum || parsed > maximum) return 0;
    *value = (int32_t)parsed;
    *cursor = delimiter ? s+1 : s;
    return 1;
}

int Control_Command(const char *command)
{
    char line[224];
    if (!strcmp(command,"CONTROL_HELLO")) {
        FSR_Transmit("#HELLO,OASIS_CONTROL,v6,yaw_min_mdeg=0,yaw_limit_mdeg=27000,pitch_limit_mdeg=8000,timeout_ms=1500,motor_rate_mdeg_s=15000,ds_outputs=4,ik_revision=8,crank_zero_us=1200,joints=2\r\n");
        return 1;
    }
    if (!strcmp(command,"ARM")) {
        Control_Heartbeat();
        armed = 1U;
        fault = "NONE";
        FSR_Transmit("#ACK,ARM\r\n");
        return 1;
    }
    if (!strcmp(command,"HOLD")) {
        update(HAL_GetTick());
        Control_Fault("HOLD");
        FSR_Transmit("#ACK,HOLD\r\n");
        return 1;
    }
    if (!strcmp(command,"STATE")) {
        status(line,sizeof line);
        FSR_Transmit(line);
        return 1;
    }
    if (strncmp(command,"SET,",4)) return 0;
    const char *cursor = command+4;
    int32_t id = 0, requested[4] = {0};
    const char *error = NULL;
    uint16_t checked[4];
    if (!number(&cursor,0,INT32_MAX,&id,',')) error = "INVALID";
    for(unsigned i=0;i<4 && !error;i++) {
        int32_t maximum=(i&1U) ? JOINT_PITCH_LIMIT_MDEG : JOINT_YAW_LIMIT_MDEG;
        if(!number(&cursor,(i&1U) ? -maximum : 0,maximum,&requested[i],i==3 ? '\0' : ',')) error="INVALID";
    }
    if (!error && !Joint_PosePulses(requested,checked)) error = "MOTOR_BOUNDS";
    if (!error && !armed) error = "DISARMED";
    if (error) {
        snprintf(line,sizeof line,"#ERR,%" PRId32 ",%s\r\n",id,error);
        FSR_Transmit(line);
        return 1;
    }
    update(HAL_GetTick());
    uint32_t left=Joint_Duration(pose[0],pose[1],requested[0],requested[1]);
    uint32_t right=Joint_Duration(pose[2],pose[3],requested[2],requested[3]);
    duration=left && right ? (left>right ? left : right) : 0;
    if (!duration) { Control_Fault("PATH_UNREACHABLE"); FSR_Transmit("#ERR,0,PATH_UNREACHABLE\r\n"); return 1; }
    memcpy(from,pose,sizeof pose);
    memcpy(target,requested,sizeof target);
    last_update = begun = HAL_GetTick();
    Control_Heartbeat();
    command_id = (uint32_t)id;
    moving = 1U;
    snprintf(line,sizeof line,"#ACK,SET,%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRIu32 "\r\n",
        id,requested[0],requested[1],requested[2],requested[3],duration);
    FSR_Transmit(line);
    return 1;
}

void Control_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE,&gpio);
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks,&latency);
    uint32_t clock = HAL_RCC_GetPCLK2Freq();
    if (clocks.APB2CLKDivider != RCC_APB2_DIV1) clock *= 2U;
    if (clock % 1000000U) { FSR_MUX_FaultDisable(); while(1){} }
    timer.Instance = TIM1;
    timer.Init.Prescaler = clock/1000000U-1U;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = 19999U;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) { FSR_MUX_FaultDisable(); while(1){} }
    TIM_OC_InitTypeDef pwm = {0};
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    for(unsigned i=0;i<4;i++) {
        pwm.Pulse = pulses[i];
        if(HAL_TIM_PWM_ConfigChannel(&timer,&pwm,channels[i]) != HAL_OK) { FSR_MUX_FaultDisable(); while(1){} }
    }
    TIM1->EGR = TIM_EGR_UG;
    for(unsigned i=0;i<4;i++) {
        if(HAL_TIM_PWM_Start(&timer,channels[i]) != HAL_OK) { FSR_MUX_FaultDisable(); while(1){} }
    }
    last_alive = last_status = HAL_GetTick();
}

void Control_Poll(void)
{
    uint32_t now = HAL_GetTick();
    if (armed && (uint32_t)(now-last_alive) >= CONTROL_TIMEOUT_MS) Control_Fault("TIMEOUT");
    update(now);
    if ((uint32_t)(now-last_status) >= 200U) {
        char line[192];
        status(line,sizeof line);
        if (FSR_TryTelemetry(line)) last_status = now;
    }
}
