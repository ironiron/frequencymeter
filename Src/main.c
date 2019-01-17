/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

volatile uint32_t k=0;//counter
volatile uint32_t tim_interrupt=0;
volatile uint32_t freq=0;
volatile uint32_t USBpacketreceived=0;
volatile float PWM_freq=0;
volatile float PWM_duty=0;
volatile uint32_t timprescaler=50;

volatile uint32_t b=0;
volatile uint32_t a=0;

volatile enum current_mode
{
	inputfrequency,
	inputPWM
}current_mode;

void TIM2_IRQHandler(void)
{
  if (current_mode == inputfrequency)
    {
      TIM4->CR1 &= ~ TIM_CR1_CEN; //Disable timers
      TIM2->CR1 &= ~ TIM_CR1_CEN;

      if (TIM4->SR & (uint16_t) 0x0001) //if timers reached TOP at the same time
	{
	  k++;
	}
      freq = (k * 0xffff + (TIM4->CNT)) * timprescaler;
      TIM2->CNT = 0;
      TIM4->CNT = 0;
      k = 0;

      TIM2->CR1 |= TIM_CR1_CEN;
      TIM4->CR1 |= TIM_CR1_CEN;
    }
  tim_interrupt = 1;
  TIM2->SR &= ~(uint16_t) 0x0001; //clear interrupt flag
}

void TIM4_IRQHandler(void)
{
  TIM4->SR &= ~(uint16_t) 0x0001; //clear interrupt flag
  k++;
}

void EXTI0_IRQHandler(void)
{
  EXTI->PR = EXTI_PR_PIF0; //clear interrupt
  USBpacketreceived = 1;
}

void tim_init_PWM(void)
{
  current_mode=inputPWM;
  /*clean registers*/
  TIM4->SMCR =0;
  TIM4->CCMR1 =0;
  TIM4->CCER =0;

  TIM4->PSC=timprescaler-1;
  TIM4->CCMR1 |=TIM_CCMR1_CC2S_0;//IC2 mapped on TI2
  TIM4->CCMR1 |=TIM_CCMR1_CC1S_1;//IC1 mapped on TI2
  TIM4->CCER  |=TIM_CCER_CC2P;//falling edge IC2
  TIM4->CCER  &=~TIM_CCER_CC1P;//rising edge IC1
  TIM4->SMCR |=TIM_SMCR_TS_1 | TIM_SMCR_TS_2 | TIM_SMCR_SMS_2;//slave mode-rest,Filtered Timer Input 2
  TIM4->CCER |=TIM_CCER_CC1E | TIM_CCER_CC2E;//Enable capture
  /*disable interrupt*/
  TIM4->DIER &=~TIM_DIER_UIE;
  TIM4->EGR = TIM_EGR_UG;//Generate an update event to reload the Prescaler
}

void tim_init_freq(void)
{
  current_mode=inputfrequency;
  /*clean registers*/
  TIM4->SMCR =0;
  TIM4->CCMR1 =0;
  TIM4->CCER =0;

  TIM4->PSC=timprescaler-1;
  TIM4->CCMR1 |=TIM_CCMR1_CC2S_0;//IC2 mapped on TI2
  TIM4->SMCR |=TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;//external clock mode 1
  TIM4->SMCR |=TIM_SMCR_TS_2 | TIM_SMCR_TS_1;//Filtered input 2
  /*enable interrupt*/
  TIM4->DIER |=TIM_DIER_UIE;
  TIM4->EGR = TIM_EGR_UG;//Generate an update event to reload the Prescaler
}

void ProcessCmd (void)
{
  uint32_t buf_len;
  uint8_t *buff = NULL; //pointer to received data
  uint8_t temp_buf[50]; //temporary buffer of received data
  int8_t index = 0;

  USBpacketreceived = 0;
  USBsend ("\n\n\r----Entered Settings mode----\n\r"
	   "Syntax: \r\n"
	   "command=value;\n\r"
	   "Type help, for available commands or exit to return.\n\r");
  USBsend ("\n\rwaiting for command>>");
  while (1)
    {
      if (USBpacketreceived == 1)
	{
	  USBpacketreceived = 0;
	  buff = USBgetdata (&buf_len);
	  temp_buf[index] = *buff;
	  CDC_Transmit_FS (buff, buf_len);
	  if (temp_buf[index] == '\r' || temp_buf[index] == '\n') //pressed enter
	    {

	      if (strncmp (temp_buf, "exit", 4) == 0) //typed exit
		{
		  return;
		}
	      else if (strncmp (temp_buf, "help", 4) == 0) //typed exit
		{
		  USBsend("\r\n---Possible commands---"
		      "\r\nPWM=<value> where value can be a number between 1-3"
		      "\r\neach number represents measureable input PWM frequency:"
		      "\r\n1:	72kHz-1,2kHz"
		      "\r\n2:	2,4kHz-40Hz"
		      "\r\n3:	1,44KHz-24Hz"
		      "\r\n"
		      "\r\nfreq=<value> where value is timer prescaler and can be a number between 1-65536"
		      "\r\nFor most cases use 1."
		      "\r\nexit to exit"
		      "\r\nhelp to show above content");
		}
	      else if (strncmp (temp_buf, "PWM=", 4) == 0) //typed exit
		{
		  timprescaler=atoi(&temp_buf[4]);
		  switch (timprescaler)
		  {
		    case 1:
		      break;
		    case 2:
		      timprescaler=30;
		      break;
		    case 3:
		      timprescaler=50;
		      break;
		    default:
		      timprescaler=1;
		      USBsend("\r\nWrong value!");
		      break;
		  }
		  tim_init_PWM();
		}
	      else if (strncmp (temp_buf, "freq=", 5) == 0) //typed exit
		{
		  timprescaler=atoi(&temp_buf[5]);
		  tim_init_freq();
		}
	      else //wrong command
		{
		  USBsend ("\r\n\n***wrong command!!!***\r\nTry again");
		}
		  for (; index >= 0; index--) //clear buffer and set index=0
		    {
		      temp_buf[index] = 0;
		    }
	      USBsend ("\n\rwaiting for command>>");
	    }
	  if(temp_buf[index]==8)//backspace pressed
	    {
	      temp_buf[index]=0;
	      if(index!=0)
		{
		  USBsend(" ");
		  USBsend_raw(8);
		  index--;
		  temp_buf[index]=0;
		}
	      else
		{
		  USBsend("\033[1C");//move cursor forward
		}
	      index--;//because in next step index is increased whenever new character arrived or just backspaced.
	    }
	  index++;
	  if(index>45)//safety feature
	    {
	      temp_buf[index]=0;
	      index=45;
	    }
	}
    }
}

int main(void)
{
  HAL_Init ();
  SystemClock_Config ();
  MX_GPIO_Init ();
  MX_TIM2_Init ();
  MX_USB_DEVICE_Init ();
  MX_TIM3_Init ();
  //MX_TIM4_Init ();
  MX_TIM1_Init ();
  /* Initialize interrupts */
  MX_NVIC_Init ();

  __HAL_RCC_TIM4_CLK_ENABLE();
  tim_init_PWM();
  HAL_TIM_Base_Start_IT (&htim2);
  TIM4->CR1 |= TIM_CR1_CEN;
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM4->DIER |=TIM_DIER_UIE;

  HAL_Delay (500);
  USBsend ("\r\n\nThis device displays frequency of signal from PB7 Pin.\r\n"
	   "Square wave outputs:\r\nPA7-1kHz\r\nPA9-1MHz\r\nPA8-8MHz");
  HAL_Delay (500);
  while (1)
    {
      if (tim_interrupt == 1)
	{
	  tim_interrupt = 0;
	  switch (current_mode)
	    {
	    case inputPWM:
	      PWM_freq = TIM4->CCR2 + 1; //temporary
	      PWM_duty = TIM4->CCR1 + 1;
	      PWM_duty = (PWM_freq - PWM_duty) * 100 / PWM_freq;
	      PWM_freq = 72000000 / timprescaler / PWM_freq;
	      USBsend ("\n\rFrequency=");
	      USBsend_Float (PWM_freq);
	      USBsend ("Hz");
	      USBsend ("\n\rDuty=");
	      USBsend_Float(PWM_duty);
	      USBsend ("%");
	      break;
	    case inputfrequency:
	      USBsend ("\n\rfrequency=");
	      USBsend_Int (freq);
	      USBsend ("Hz");

	      break;
	    }
	}
      if (USBpacketreceived == 1)
	{
	  USBpacketreceived = 0;
	  TIM4->CR1 &= ~ TIM_CR1_CEN; //Disable timers
	  TIM2->CR1 &= ~ TIM_CR1_CEN;

	  ProcessCmd ();

	  k = 0;
	  TIM4->CNT = 0;
	  TIM2->CNT = 0;
	  TIM2->CR1 |= TIM_CR1_CEN;
	  TIM4->CR1 |= TIM_CR1_CEN;
	}
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = 8;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  //USB interrupt priority 2.

  /*This interrupt is used for informing about USB DATA OUT */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  EXTI->IMR=EXTI_IMR_IM0;
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3-1;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	  TIM_ClockConfigTypeDef sClockSourceConfig;
	  TIM_OC_InitTypeDef sConfigOC;

	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 72-1;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 1000-1;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 500;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  HAL_TIM_MspPostInit(&htim3);

	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins
     PA8   ------> RCC_MCO
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
