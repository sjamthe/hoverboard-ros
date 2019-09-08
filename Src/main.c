/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include <string.h>

void SystemClock_Config(void);
void Error_Handler(void);
void motor_init(void);

void ros_init(void);
void ros_run(void);

//#define CONTROL_SERIAL_USART2_DMA

#ifdef CONTROL_SERIAL_USART2
  extern UART_HandleTypeDef huart2; 
  uint8_t ch_buf[10];
#ifdef CONTROL_SERIAL_USART2_DMA
	extern DMA_HandleTypeDef hdma_usart2_rx;
	extern DMA_HandleTypeDef hdma_usart2_tx;
#endif
#endif

#ifdef CONTROL_MOTOR_TEST
  extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
  extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
#endif
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

uint16_t speed = 0;
char message[100];
uint16_t counter = 0;

void poweroff() {
    if (ABS(speed) < 20) {
        buzzerPattern = 0;
        for (int i = 0; i < 8; i++) {
            buzzerFreq = i;
            HAL_Delay(100);
        }
        HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
        while(1) {}
    }
}


int main(void) {
  HAL_Init();

  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();
  __HAL_RCC_DMA1_CLK_DISABLE();

  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  #ifdef CONTROL_SERIAL_USART2
    UART_Control_Init();
  #endif
  #ifdef CONTROL_SERIAL_USART2_DMA
    if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)&ch_buf, 10)  != HAL_OK) 
    {
      Error_Handler();
    }
  #endif

  #ifdef DEBUG_SERIAL_USART3
    UART_Debug_Init();
  #endif

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

 Hall_Sensor_Init();
 #ifdef HALL_INTERRUPTS
  HallInterruptinit();
 #endif

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  motor_init();
  ros_init();

  // ###### STARTUP CHIME #############
  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
 
#ifdef CONTROL_MOTOR_TEST

	#ifdef INVERT_R_DIRECTION
	  pwmr = 60;
	#else
	  pwmr = -60;
	#endif
	#ifdef INVERT_L_DIRECTION
	  pwml = -60;
	#else
	  pwml = 60;
	#endif

#endif
//invert direction
pwml = -1*pwml;
//pwmr = -1*pwmr;

  while(1) {
    HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms
    ros_run();

   // ####### POWEROFF BY POWER-BUTTON #######
   if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
     while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}
     poweroff();
   }
  }
}

#ifdef CONTROL_SERIAL_USART2_DMA
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
#endif

/** System Clock Configuration
*/
void SystemClock_Config(void) {

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  for (int i = 2; i >= 0; i--) {
    buzzerFreq = 4;
    buzzerPattern = 1;
    HAL_Delay(5);
  }
  buzzerFreq = 0;
}
