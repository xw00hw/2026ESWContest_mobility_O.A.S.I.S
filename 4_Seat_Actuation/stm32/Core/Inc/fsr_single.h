#ifndef FSR_SINGLE_H
#define FSR_SINGLE_H
#include "stm32h7xx_hal.h"
void FSR_Init(ADC_HandleTypeDef *adc, UART_HandleTypeDef *uart);
void FSR_Poll(void);
#endif
