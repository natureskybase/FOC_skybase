/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lptim.c
  * @brief   This file provides code for the configuration
  *          of the LPTIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "lptim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* LPTIM1 init function */
void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**LPTIM1 GPIO Configuration
  PB2   ------> LPTIM1_OUT
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV1);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);
  LL_LPTIM_SetInput1Src(LPTIM1, LL_LPTIM_INPUT1_SRC_GPIO);
  LL_LPTIM_SetInput2Src(LPTIM1, LL_LPTIM_INPUT2_SRC_GPIO);
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
