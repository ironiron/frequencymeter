/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Virtual Com Port Demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include <stdio.h>
#include "delay.h"

extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
__IO uint32_t packet_sent;

volatile uint32_t k=0;
volatile uint32_t freq=0;

void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        k++;
        GPIOC->ODR ^=(1<<13);
    }
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM_Cmd(TIM3, DISABLE);
        freq=(k*0xffff+(TIM3->CNT))*2;
        k=0;
        TIM3->CNT=0;
        TIM_Cmd(TIM3, ENABLE);
    }
}

int main(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    NVIC_SetPriority(SysTick_IRQn,2);
    SysTick_Config(SystemCoreClock / 1000);

    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;
    TIM_ICInitTypeDef channel;
    TIM_OCInitTypeDef  pwm;

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin=GPIO_Pin_7;/* signal IN */
    gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&gpio);

    gpio.GPIO_Pin=GPIO_Pin_13;/* LED */
    gpio.GPIO_Mode=GPIO_Mode_Out_PP;
    gpio.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&gpio);
    /**************************************************************************/
    gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);
	RCC_MCOConfig(RCC_MCO_HSE);/* output clock */
    /**************************************************************************/
    /* Timer 3 is clocked by  external signal */
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Prescaler = 2- 1;
    tim.TIM_Period = 0xffff;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_ICStructInit(&channel);
    channel.TIM_Channel=TIM_Channel_2;
    channel.TIM_ICSelection=TIM_ICSelection_DirectTI;
    channel.TIM_ICPolarity=TIM_ICPolarity_Rising;
    channel.TIM_ICFilter=0x00;
    channel.TIM_ICPrescaler=TIM_ICPSC_DIV8;
    TIM_ICInit(TIM3,&channel);

    TIM_TIxExternalClockConfig(TIM3,TIM_TIxExternalCLK1Source_TI2,TIM_ICPolarity_Rising,0x00);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    /**************************************************************************/
    /* Timer 2 is used for computing data every 1s. */
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Prescaler = 64000 - 1;
    tim.TIM_Period = 1000 - 1;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    /**************************************************************************/
    /*TIM4 is used to generate 1kHz */
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Prescaler = 64 - 1;
    tim.TIM_Period = 1000 - 1;
    TIM_TimeBaseInit(TIM4, &tim);

    TIM_OCStructInit(&pwm);
    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = 500;
    TIM_OC2Init(TIM4, &pwm);

    TIM_Cmd(TIM4, ENABLE);
    /**************************************************************************/
    /*TIM1 is used to generate 1MHz */
   /* gpio.GPIO_Pin = GPIO_Pin_14;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Prescaler = 16 - 1;
    tim.TIM_Period = 4 - 1;
    TIM_TimeBaseInit(TIM1, &tim);

    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = 2;
    TIM_OC2Init(TIM1, &pwm);

    TIM_Cmd(TIM1, ENABLE);
*/
    _delay_ms(500);
	USBsend("This device displays frequency of input signal from PA7 Pin.\r\n"
            "Result is displayed in Hz.\r\n"
            "Square wave outputs on:\r\nPB7-1kHz\r\nPB14-1MHz");
	_delay_ms(500);
    while (1)
    {
        _delay_ms(1000);
        USBsend("\n\rfrequency=");
        USBsend_Int(freq);
    }
}
