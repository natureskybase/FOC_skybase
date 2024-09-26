/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "delay.h"
#include "FOC.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 
  // Reset USB CDC and stm32 wakeup
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_LPTIM1_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  delay_init(170);
  //stm32 wakeup
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

	// ADC_DMA_Start();
  LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_1,LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),(uint32_t)ADC_ConvertedValue,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);//配置DMA,将DMA与ADC1链接到一�??????????
	LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,12);
  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
	LL_ADC_REG_SetDMATransfer(ADC1,LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_StartCalibration(ADC1,LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1));   //等待校准完成
	LL_ADC_Enable(ADC1);
	while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
	LL_ADC_REG_StartConversion(ADC1);


  // //spi_dma配置
  // uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
  //   // LL_DMA_ConfigTransfer(DMA1,
  //   //                     LL_DMA_CHANNEL_3,
  //   //                     LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
  //   //                     LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
  //   //                     LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD);
  // LL_DMA_ConfigAddresses(DMA1,
  //                        LL_DMA_CHANNEL_3,
  //                        (uint32_t)aTxBuffer, LL_SPI_DMA_GetRegAddr(SPI1),
  //                        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, ubNbDataToTransmit);
  // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_SPI1_TX);

  // uint8_t ubNbDataToReceive = sizeof(aRxBuffer);
  // //  LL_DMA_ConfigTransfer(DMA1,
  // //                       LL_DMA_CHANNEL_2,
  // //                       LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
  // //                       LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
  // //                       LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD);
  // LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)aRxBuffer,
  //                        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
  // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ubNbDataToReceive);
  // LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_SPI1_RX);

  // LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  // LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
  // LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  // LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

  // /* Initialize FFIFO Threshold */
  // LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

  // /* Configure SPI1 DMA transfer interrupts */
  // /* Enable DMA RX Interrupt */
  // LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  // LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  // LL_SPI_EnableDMAReq_RX(SPI1);
  // LL_SPI_EnableDMAReq_TX(SPI1);
  /* Enable the SPI1 peripheral */
  // LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2);
  // LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_3);
  LL_SPI_Enable(SPI1);
  
  // LL_SPI_Enable(SPI1);

  FOC_init(FOC_MOTOR); 
  Current_get_init();

  LL_mDelay(100);
  /*使能TIM1时钟，使其可以进入中断中*/
	LL_TIM_ClearFlag_UPDATE(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1); //TIM1更新使能

  /*使能TIM1时钟，使其可以进入中断中*/
  LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableCounter(TIM3);   //TIM3计数使能
  LL_TIM_EnableIT_UPDATE(TIM3); //TIM3更新使能

  // struct 
  // {
  //   float x[7];
  //   unsigned char tail[4];
  // } x={.tail = {0x00, 0x00, 0x80, 0x7f}};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Theta_get(FOC_MOTOR);
    // Svpwm_sensor(1,FOC_MOTOR->theta);
    // Svpwm(FOC_MOTOR,0.5,0,FOC_MOTOR->theta);
    // Svpwm(FOC_MOTOR,2,0,FOC_MOTOR->theta);
    // Svpwm(FOC_MOTOR,1,0,4.71);
    // Pwm_change(DA,DB,DC);

    Theta_get(FOC_MOTOR);
    Current_get(FOC_MOTOR);
    Svpwm(FOC_MOTOR,1 ,0,FOC_MOTOR->theta);
    Pwm_change(DA,DB,DC);

    // x.x[0]=FOC_MOTOR->voltage_info[0];
    // x.x[1]=FOC_MOTOR->voltage_info[1];
    // x.x[2]=FOC_MOTOR->voltage_info[2];
    // CDC_Transmit_FS(&x, sizeof(x));
    // USBVcom_printf("%f, %f, %f\r\n", DA, DB, DC);
		// USBVcom_printf("%f, %f, %f,%f,%f,%f\r\n", DA, DB, DC,FOC_MOTOR->voltage_info[0], FOC_MOTOR->voltage_info[1], FOC_MOTOR->voltage_info[2]);
    // USBVcom_printf("%f, %f, %f\r\n", FOC_MOTOR->voltage_info[0], FOC_MOTOR->voltage_info[1], FOC_MOTOR->voltage_info[2]);
    // USBVcom_printf("%d, %d, %d\r\n", ADC_ConvertedValue[0], ADC_ConvertedValue[1], ADC_ConvertedValue[2]);
    // USBVcom_printf("%f, %f\r\n", FOC_MOTOR->current_q, FOC_MOTOR->current_d);
    // USBVcom_printf("%f, %f, %d\r\n",FOC_MOTOR->theta,FOC_MOTOR->machanical_theta,FOC_MOTOR->angle_raw);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_HSI48_Enable();
   /* Wait till HSI48 is ready */
  while(LL_RCC_HSI48_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_5, 68, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
