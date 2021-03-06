/**
 **************************************************************************
 * File Name    : at32f4xx_it.c
 * Description  : at32f4xx interrupt service routines.
 * Date         : 2018-02-12
 * Version      : V1.0.4
 **************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "../Inc/stm32f1xx_it.h"
#include "config.h"
#include "setup.h"
#include "hallinterrupts.h"

#ifdef CONTROL_SERIAL_USART2
	extern DMA_HandleTypeDef hdma_usart2_rx;
	extern DMA_HandleTypeDef hdma_usart2_tx;
#endif

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// void SysTick_Handler(void)
// {

// }

/**
* @brief This function handles System tick timer.
*/
// #ifdef CONTROL_PPM
// void PPM_SysTick_Callback(void);
// #endif
void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
#ifdef CONTROL_PPM
  PPM_SysTick_Callback();
#endif
  /* USER CODE END SysTick_IRQn 1 */
}

#ifdef CONTROL_NUNCHUCK
extern I2C_HandleTypeDef hi2c2;
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c2);
}

void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c2);
}

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}
#endif

#ifdef CONTROL_PPM
void EXTI3_IRQHandler(void)
{
    PPM_ISR_Callback();
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
}
#endif

////////////////////////////////////////////////////////////////////
// actual IRQ for LEFT pins 5,6,7
void EXTI9_5_IRQHandler(void)
{
  unsigned long triggered = 0;
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
  {
    /* Clear the EXTI line 8 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
    triggered |= GPIO_PIN_9;
  }
  
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
  {
    /* Clear the EXTI line 9 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    triggered |= GPIO_PIN_9;
  }

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    triggered |= GPIO_PIN_7;
  }

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    triggered |= GPIO_PIN_6;
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    triggered |= GPIO_PIN_5;
  }
 
#ifdef HALL_INTERRUPTS
  if (triggered & HALL_PIN_MASK)
    HallInterruptsInterrupt();
#endif

// shared interrupt for these pins, depending on where the sfotware serial pin is
#ifdef SOFTWARE_SERIAL
  if (triggered & SOFTWARE_SERIAL_RX_PIN){
      softwareserialRXInterrupt();
  }
#endif
} 

/////////////////////////////////////////////////////////////////////
// actual IRQ for RIGHT pins 10, 11, 12
void EXTI15_10_IRQHandler(void)
{
  unsigned long triggered = 0;
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
  {
    /* Clear the EXTI line 8 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
    triggered |= GPIO_PIN_15;
  }
  
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
  {
    /* Clear the EXTI line 9 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
    triggered |= GPIO_PIN_14;
  }

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    triggered |= GPIO_PIN_13;
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    triggered |= GPIO_PIN_12;
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    triggered |= GPIO_PIN_11;
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
    triggered |= GPIO_PIN_10;
  }

#ifdef HALL_INTERRUPTS
 if (triggered & HALL_PIN_MASK)
   HallInterruptsInterrupt();
#endif

} 

#ifdef CONTROL_SERIAL_USART2
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}
#endif
