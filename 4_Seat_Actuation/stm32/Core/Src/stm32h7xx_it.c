/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "stm32h7xx_it.h"

extern UART_HandleTypeDef huart3;

void NMI_Handler(void)
{

  FSR_MUX_FaultDisable();

   while (1)
  {
  }

}

void HardFault_Handler(void)
{

  FSR_MUX_FaultDisable();

  while (1)
  {

  }
}

void MemManage_Handler(void)
{

  FSR_MUX_FaultDisable();

  while (1)
  {

  }
}

void BusFault_Handler(void)
{

  FSR_MUX_FaultDisable();

  while (1)
  {

  }
}

void UsageFault_Handler(void)
{

  FSR_MUX_FaultDisable();

  while (1)
  {

  }
}

void SVC_Handler(void)
{

}

void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}

void SysTick_Handler(void)
{

  HAL_IncTick();

}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}
