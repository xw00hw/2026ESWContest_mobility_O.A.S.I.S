/* Opt-in pressure-only control backend. The motion controller/kinematics are
 * not linked into this image. D3/D4/D5/D6 never become PWM alternate functions.
 * MUX D8..D12 and ADC A0/A1/A2 are owned by the unchanged FSR implementation. */
#ifndef OASIS_PRESSURE_ONLY
#error "pressure_only_control.c requires the explicit pressure-only build option"
#endif
#include "stm32h7xx_hal.h"
#include "joint_control.h"
#include "pressure_only_control.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#define MOTOR_PINS (GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14)
#define MOTOR_MODE_MASK ((3u<<18)|(3u<<22)|(3u<<26)|(3u<<28))
#define MOTOR_OUTPUT_MODE ((1u<<18)|(1u<<22)|(1u<<26)|(1u<<28))
#define MOTOR_AF_MASK ((15u<<4)|(15u<<12)|(15u<<20)|(15u<<24))

void PressureOnly_MotorOff(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->AHB4ENR; (void)RCC->APB2ENR;
    TIM1->BDTR=0;
    TIM1->CCER=0;
    TIM1->CR1=0;
    TIM1->DIER=0;
    TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=0;
    GPIOE->BSRR=(uint32_t)MOTOR_PINS<<16;
    GPIOE->OTYPER &= ~(uint32_t)MOTOR_PINS;
    GPIOE->PUPDR &= ~MOTOR_MODE_MASK;
    GPIOE->AFR[1] &= ~MOTOR_AF_MASK;
    GPIOE->MODER=(GPIOE->MODER & ~MOTOR_MODE_MASK)|MOTOR_OUTPUT_MODE;
    __DSB();
}

void Control_Init(void) { PressureOnly_MotorOff(); }
void Control_Poll(void) { PressureOnly_MotorOff(); }
void Control_Heartbeat(void) { /* FSR owns its independent scan timeout. */ }
void Control_Fault(const char *reason)
{
    (void)reason;
    PressureOnly_MotorOff();
}

int Control_Command(const char *command)
{
    /* Reassert LOW before handling even malformed or motor-directed input. */
    PressureOnly_MotorOff();
    if (!strcmp(command,"CONTROL_HELLO")) {
        FSR_Transmit("#HELLO,OASIS_PRESSURE_ONLY,v1,bits=12,hz=100,banks=3,max_channels=16,motor_outputs=0,pwm=OFF\r\n");
        return 1;
    }
    if (!strcmp(command,"ARM") || !strcmp(command,"SET") || !strncmp(command,"SET,",4)) {
        FSR_Transmit("#ERR,0,PRESSURE_ONLY\r\n");
        return 1;
    }
    if (!strcmp(command,"HOLD")) {
        FSR_Transmit("#ACK,HOLD\r\n");
        return 1;
    }
    if (!strcmp(command,"STATE")) {
        char line[128];
        snprintf(line,sizeof line,
            "#STATE,%" PRIu32 ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,PRESSURE_ONLY\r\n",HAL_GetTick());
        FSR_Transmit(line);
        return 1;
    }
    return 0; /* HELLO/START_12/PING/DISABLE remain owned by FSR. */
}
