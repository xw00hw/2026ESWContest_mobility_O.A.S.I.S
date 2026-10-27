/* Dedicated opt-in output boundary; never touches the pressure mux pins.
 * D3/D4/D5/D6 = PE13/14/11/9 = TIM1 CH3/4/2/1, AF1, 50 Hz.
 * Register readback is not waveform/shaft-position feedback. */
#ifndef OASIS_FALLBACK_CONTROL
#error "fallback_pwm requires OASIS_FALLBACK_CONTROL"
#endif
#include "stm32h7xx_hal.h"
#include "fallback_control.h"

#define PINS (GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14)
#define MODE_MASK ((3u<<18)|(3u<<22)|(3u<<26)|(3u<<28))
#define OUTPUT_MODE ((1u<<18)|(1u<<22)|(1u<<26)|(1u<<28))
#define AF_MODE (OUTPUT_MODE<<1)
#define AF_MASK ((15u<<4)|(15u<<12)|(15u<<20)|(15u<<24))
#define AF_VALUE ((1u<<4)|(1u<<12)|(1u<<20)|(1u<<24))
#define ENABLES (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E)
#define PWM12 (TIM_CCMR1_OC1PE|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC2PE|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2)
#define PWM34 (TIM_CCMR2_OC3PE|TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC4PE|TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2)

void Fallback_MotorOff(void)
{
    const uint32_t saved=__get_PRIMASK();
    __disable_irq();
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->AHB4ENR; (void)RCC->APB2ENR;
    TIM1->BDTR=0; TIM1->CCER=0; TIM1->CR1=0; TIM1->DIER=0;
    TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=TIM1->CCR4=0;
    GPIOE->BSRR=(uint32_t)PINS<<16;
    GPIOE->OTYPER &= ~(uint32_t)PINS;
    GPIOE->PUPDR &= ~MODE_MASK;
    GPIOE->AFR[1] &= ~AF_MASK;
    GPIOE->MODER=(GPIOE->MODER & ~MODE_MASK)|OUTPUT_MODE;
    __DSB();
    __set_PRIMASK(saved);
}

static bool clock_valid(void)
{
    return SystemCoreClock==64000000U && HAL_RCC_GetPCLK2Freq()==64000000U &&
        (RCC->AHB4ENR & RCC_AHB4ENR_GPIOEEN) && (RCC->APB2ENR & RCC_APB2ENR_TIM1EN);
}

bool Fallback_PwmValid(const uint16_t p[4], bool armed)
{
    if (TIM1->CCR3!=p[0] || TIM1->CCR4!=p[1] || TIM1->CCR2!=p[2] || TIM1->CCR1!=p[3]) return false;
    if (!armed) return !p[0] && !p[1] && !p[2] && !p[3] && !TIM1->CCER &&
        !(TIM1->BDTR & TIM_BDTR_MOE) && !(TIM1->CR1 & TIM_CR1_CEN) &&
        (GPIOE->MODER & MODE_MASK)==OUTPUT_MODE && !(GPIOE->ODR & PINS) && !(GPIOE->IDR & PINS);
    return clock_valid() && TIM1->CR1==(TIM_CR1_CEN|TIM_CR1_ARPE) &&
        TIM1->CR2==0 && TIM1->SMCR==0 && TIM1->DIER==0 && TIM1->PSC==63 &&
        TIM1->ARR==19999 && TIM1->RCR==0 && TIM1->CCMR1==PWM12 && TIM1->CCMR2==PWM34 &&
        TIM1->CCER==ENABLES && TIM1->BDTR==TIM_BDTR_MOE &&
        (GPIOE->MODER & MODE_MASK)==AF_MODE && (GPIOE->AFR[1] & AF_MASK)==AF_VALUE &&
        !(GPIOE->OTYPER & PINS) && !(GPIOE->PUPDR & MODE_MASK);
}

bool Fallback_PwmStart(const uint16_t p[4])
{
    const uint16_t zero[4]={0,0,0,0};
    if (!Fallback_PwmValid(zero,false) || !clock_valid()) return false;
    for (unsigned i=0;i<4;i++) if (p[i]<500 || p[i]>2500) return false;
    const uint32_t saved=__get_PRIMASK();
    __disable_irq();
    GPIOE->OSPEEDR &= ~MODE_MASK;
    GPIOE->AFR[1]=(GPIOE->AFR[1]&~AF_MASK)|AF_VALUE;
    TIM1->CR1=TIM_CR1_ARPE; TIM1->CR2=0; TIM1->SMCR=0; TIM1->DIER=0;
    TIM1->PSC=63; TIM1->ARR=19999; TIM1->RCR=0; TIM1->CNT=0;
    TIM1->CCMR1=PWM12; TIM1->CCMR2=PWM34;
    /* OFF-only UG latches zero. First natural overflow starts all four full
       neutral pulses together; enabling channels does not emit a short pulse. */
    TIM1->EGR=TIM_EGR_UG; TIM1->SR=0;
    TIM1->CCR3=p[0]; TIM1->CCR4=p[1]; TIM1->CCR2=p[2]; TIM1->CCR1=p[3];
    DBGMCU->APB2FZ1 &= ~DBGMCU_APB2FZ1_DBG_TIM1;
    TIM1->CCER=ENABLES; TIM1->BDTR=TIM_BDTR_MOE;
    GPIOE->MODER=(GPIOE->MODER&~MODE_MASK)|AF_MODE;
    TIM1->CR1=TIM_CR1_ARPE|TIM_CR1_CEN;
    __DSB(); __set_PRIMASK(saved);
    return Fallback_PwmValid(p,true);
}

bool Fallback_PwmFrameReady(void)
{
    /* One stage per natural timer frame; leave 3 ms before rollover and wait
       until all allowed <=2500 us pulses have ended. Never catch up missed frames. */
    uint32_t count=TIM1->CNT;
    return (TIM1->SR & TIM_SR_UIF) && count>=3000U && count<=17000U;
}

bool Fallback_PwmStage(const uint16_t p[4])
{
    for (unsigned i=0;i<4;i++) if (p[i]<500 || p[i]>2500) return false;
    const uint32_t saved=__get_PRIMASK();
    __disable_irq();
    if (!Fallback_PwmFrameReady()) { __set_PRIMASK(saved); return false; }
    /* IRQ masking alone cannot prevent a timer overflow between CCR writes. */
    TIM1->CR1 |= TIM_CR1_UDIS;
    __DSB(); (void)TIM1->CR1;
    TIM1->CCR3=p[0]; TIM1->CCR4=p[1]; TIM1->CCR2=p[2]; TIM1->CCR1=p[3];
    __DSB();
    TIM1->SR=0;
    TIM1->CR1 &= ~TIM_CR1_UDIS;
    __DSB(); __set_PRIMASK(saved);
    return true;
}

bool Fallback_WatchdogStart(void)
{
    /* Nominal 2 s at 32 kHz LSI: /64 * (999+1). Actual oscillator tolerance
       is not measured. It remains active during debug halt; reset boots OFF. */
    RCC->CSR |= RCC_CSR_LSION;
    uint32_t start=HAL_GetTick(), guard=2000000U;
    while (!(RCC->CSR & RCC_CSR_LSIRDY))
        if (!--guard || (uint32_t)(HAL_GetTick()-start)>=100U) return false;
    DBGMCU->APB4FZ1 &= ~DBGMCU_APB4FZ1_DBG_IWDG1;
    DBGMCU->APB2FZ1 &= ~DBGMCU_APB2FZ1_DBG_TIM1;
    IWDG1->KR=0xCCCC; IWDG1->KR=0x5555;
    IWDG1->PR=4; IWDG1->RLR=999; IWDG1->WINR=4095;
    start=HAL_GetTick(); guard=2000000U;
    while (IWDG1->SR)
        if (!--guard || (uint32_t)(HAL_GetTick()-start)>=100U) return false;
    IWDG1->KR=0xAAAA;
    return true;
}

void Fallback_WatchdogRefresh(void) { IWDG1->KR=0xAAAA; }
