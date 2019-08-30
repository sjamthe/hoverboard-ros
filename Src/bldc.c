/*
* This file has been re-implemented with 4 new selectable motor control methods.
* Recommended control method: 3 = Sinusoidal 3rd order. This control method offers superior performanace
* compared to previous method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019 Emanuel FERU <aerdronix@gmail.com>
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
#include "rtwtypes.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
// #include "BLDC_controller.h"           /* Model's header file */
// #include "rtwtypes.h"

// extern RT_MODEL *const rtM_Left;
// extern RT_MODEL *const rtM_Right;

// extern DW rtDW_Left;                    /* Observable states */
// extern ExtU rtU_Left;                   /* External inputs */
// extern ExtY rtY_Left;                   /* External outputs */

// extern DW rtDW_Right;                   /* Observable states */
// extern ExtU rtU_Right;                  /* External inputs */
// extern ExtY rtY_Right;                  /* External outputs */
// ###############################################################################


// #ifdef CONTROL_DETECT_HALL
// 	volatile uint8_t hall_idx_left = 0;
// 	volatile uint8_t hall_idx_right = 0;
// #else
// 	const uint8_t hall_idx_left  = HALL_IDX_LEFT-1;
// 	const uint8_t hall_idx_right = HALL_IDX_RIGHT-1;
// #endif

void motor_run(void);

extern volatile adc_buf_t adc_buffer;

// extern volatile uint32_t timeout;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
static uint32_t buzzerTimer = 0;

// uint8_t enable          = 0;

volatile unsigned  bldc_count_per_hall_counter[2] = {0,0};

// static int offsetcount = 0;
// static int offsetrl1   = 2000;
// static int offsetrl2   = 2000;
// static int offsetrr1   = 2000;
// static int offsetrr2   = 2000;
// static int offsetdcl   = 2000;
// static int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

//scan 8 channels (is it 10?) with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler(void) {

  DMA1->IFCR = DMA_IFCR_CTCIF1;

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99f + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01f;
  }

  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }


  static boolean_T OverrunFlag = false;
  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;
  
  motor_run();

  /* Indicate task complete */
  OverrunFlag = false;
}
