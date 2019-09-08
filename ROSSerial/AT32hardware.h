/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef STM32HARDWARE_H_
#define STM32HARDWARE_H_

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "setup.h"

extern UART_HandleTypeDef huart2; 

extern uint8_t g_RxBuf[];
extern uint8_t byte;		/* receive byte */
extern uint8_t g_RxRead;
extern uint8_t g_RxWritten;

class STM32Hardware
{
public:
	STM32Hardware()
	{
	}

	/*
	 * Method: init
	 * ----------------------------
	 *   Initialize the STM32 hardware specific to ROS like UART
	 *   parameters: none
	 *   returns: none
	 */
	void init(void)
	{
		if(HAL_UART_Receive_IT(&huart2, &byte, 1) != 0)
		{
			return ;
		}
	}

	/*
     * Method: read
	 * ----------------------------
	 *   Read a byte of data from ROS connection
	 *   parameters: none
	 *   returns: data, or -1 if there are no data
	 */
	int read(void)
	{
		int ucData = -1;
		
		if(g_RxRead < g_RxWritten)
		{
			ucData = g_RxBuf[g_RxRead++];
		}
		else if (g_RxWritten < RX_BUF_SIZE &&  g_RxRead == g_RxWritten )
		{
			g_RxRead = 0;
			g_RxWritten = 0;
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
      		HAL_UART_Receive_IT(&huart2, &byte, 1);
		}
		else if (g_RxWritten < RX_BUF_SIZE && g_RxWritten > g_RxRead)
		{
			printf("How did this happen? %d %d\n",g_RxRead,g_RxWritten);
		}
		
		
		if(g_RxRead == RX_BUF_SIZE )
		{
			g_RxRead = 0;
			g_RxWritten = 0;
			//Enable the interrupt so we can read again.
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
      		HAL_UART_Receive_IT(&huart2, &byte, 1);
		}
		return ucData;
	}

	/*
	 * Method: write
     * ----------------------------
	 *   Send data to ROS connection
	 *   parameters: pointer to data array, length of data
	 *   returns: none
	 */
	void write(uint8_t* data, uint32_t length)
	{
		// printf("printing %d\n",length);
		/* Without Interrupt we can still publish chatter at 111.6Hz or 9ms per message */
		HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t *)data, length, 10);
		if(ret == HAL_TIMEOUT)
		{
			//printf("timeout %d - %s\n",length, data);
		}
		
		// /* Try Interrupt - Has delayed byte problem. */
		// if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)data, length)!= HAL_OK)
  		// {
		// 	HAL_Delay(10); 
		// 	printf("delayed bytes %d\n",length); 
		// }

		// else
		// {
		// 	printf("sent %d bytes over Interrupt\n",length);
		// }
		
	}

	/*
	 * Method: time
	 * ----------------------------
     *   Returns milliseconds since start of program
	 *   parameters: none
	 *   returns: time in milliseconds
	 */
	uint32_t time(void)
	{
		//return (uint32_t) xTaskGetTickCount();
		return HAL_GetTick();
	}

};

#endif /* STM32H7HARDWARE_H_ */
