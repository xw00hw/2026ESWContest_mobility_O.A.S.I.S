#ifndef FSR_BOARD_PINS_H
#define FSR_BOARD_PINS_H
#include "stm32h7xx_hal.h"
#ifdef OASIS_FALLBACK_CONTROL
#include "fallback_control.h"
#endif

/* NUCLEO-H723ZG default solder bridges. PWM D3..D6 remain dedicated to servos.
   MUX S0=D8/PF3, S1=D9/PD15, S2=D10/PD14, S3=D11/PB5.
   D11 is PB5 with the default bridges, not PA7. EN=D12/PA6. */
#define FSR_MUX_EN_PORT GPIOA
#define FSR_MUX_EN_PIN GPIO_PIN_6
#define FSR_MUX_EN_MODE_SHIFT 12U

static inline void FSR_MUX_WriteAddress(uint8_t channel)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, (channel & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uint32_t selected = ((channel & 2U) ? GPIO_PIN_15 : 0U) |
                        ((channel & 4U) ? GPIO_PIN_14 : 0U);
    GPIOD->BSRR = selected | ((uint32_t)((GPIO_PIN_14 | GPIO_PIN_15) & ~selected) << 16U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (channel & 8U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void FSR_MUX_InitAddress(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    FSR_MUX_WriteAddress(0U);
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOF, &gpio);
    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* Fatal state also removes servo PWM; this does not cut external motor power. */
static inline void FSR_MUX_FaultDisable(void)
{
#ifdef OASIS_FALLBACK_CONTROL
    Fallback_MotorOff();
#endif
    __disable_irq();
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    FSR_MUX_EN_PORT->BSRR = FSR_MUX_EN_PIN;
    FSR_MUX_EN_PORT->OTYPER &= ~(uint32_t)FSR_MUX_EN_PIN;
    FSR_MUX_EN_PORT->PUPDR &= ~(3UL << FSR_MUX_EN_MODE_SHIFT);
    FSR_MUX_EN_PORT->MODER =
        (FSR_MUX_EN_PORT->MODER & ~(3UL << FSR_MUX_EN_MODE_SHIFT)) |
        (1UL << FSR_MUX_EN_MODE_SHIFT);
    __DSB();
}
#endif
