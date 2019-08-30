#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "stdio.h"
#include "string.h"

UART_HandleTypeDef huart2;

#ifdef DEBUG_SERIAL_USART3

#define UART_DMA_CHANNEL DMA1_Channel2
/* consoleLog uses DMA for heavy writes */
void consoleLog(char *message)
{
  #ifdef DEBUG_SERIAL_USART3
    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = strlen((char *)message);
      UART_DMA_CHANNEL->CMAR  = (uint32_t)message;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
  #endif
}

#endif // End of DEBUG_SERIAL_USART3
