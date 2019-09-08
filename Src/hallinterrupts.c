/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
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
#include "config.h"
#include "hallinterrupts.h"
#include <string.h>
#include "control_structures.h"

#ifdef HALL_INTERRUPTS

//////////////////////////////////////////////////////////////
// file reads Hall sensors, and gets Distance and Speed.
// Uses:
// EXTI15_10_IRQHandler
// EXTI9_5_IRQHandler
// TIM4 running at 100khz, reloading 0xFFFF
// TIM4_IRQHandler
//////////////////////////////////////////////////////////////
// Usage: just start with HallInterruptinit(void) and
// read results from HALL_DATA_STRUCT HallData[2];
//////////////////////////////////////////////////////////////
// Notes:
// for distance measurement and speed calculations, it defaults to 6.5" wheels.
// this may be changed by calling:
// void HallInterruptSetWheelDiameterInches(float inches);
// or
// void HallInterruptSetWheelDiameterMM(float mm);
//
// if you wish to reset the values in the structure completely, use:
// void HallInterruptReset();
// if you wish to read distance and speed, and optionally reset distance to zero,
// then use:
// void HallInterruptReadPosn( HALL_POSN *p, int Reset );
//
// NOTE: if the hall pins are reconfigured to NOT be in the set 5-9, 10-15
// then mods will need to be made.
//////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////
// this is the Hall data we gather, and can be read directly elsewhere
// it CAN be read with interrupts disabled using
volatile HALL_DATA_STRUCT HallData[2];

volatile uint8_t hall_ul;
volatile uint8_t hall_vl;
volatile uint8_t hall_wl;

volatile uint8_t hall_ur;
volatile uint8_t hall_vr;
volatile uint8_t hall_wr;

//////////////////////////////////////////////////////////////
// local data
TIM_HandleTypeDef h_timer_hall;
volatile HALL_PARAMS local_hall_params[2];

volatile long long timerwraps = 0;

static float WheelSize_mm = (DEFAULT_WHEEL_SIZE_INCHES * 25.4);



//////////////////////////////////////////////////////////////
// intialisation for interrupts from hall sensor edges
void HallInterruptinit(void){
    memset((void *)&HallData, 0, sizeof(HallData));
    memset((void *)&local_hall_params, 0, sizeof(local_hall_params));
    #ifdef INVERT_L_DIRECTION
        local_hall_params[LEFT].direction = -1;
    #else
        local_hall_params[LEFT].direction = 1;
    #endif
    #ifdef INVERT_R_DIRECTION
        local_hall_params[RIGHT].direction = 1;
    #else
        local_hall_params[RIGHT].direction = -1;
    #endif

    // overrides local fle default
    #ifdef WHEEL_SIZE_INCHES
    WheelSize_mm = (WHEEL_SIZE_INCHES * 25.4);
    #endif

    HallData[0].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
    HallData[1].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);

    // setup TIM4:
    __HAL_RCC_TIM4_CLK_ENABLE();
    h_timer_hall.Instance = TIM4;
    //h_timer_hall.Init.Prescaler         = 64000000 / 2 / HALL_INTERRUPT_TIMER_FREQ;
    h_timer_hall.Init.Prescaler         = 72000000 / 2 / HALL_INTERRUPT_TIMER_FREQ;
    h_timer_hall.Init.CounterMode       = TIM_COUNTERMODE_UP;
    h_timer_hall.Init.Period            = 0xFFFF; // we just want the timer to wrap
    h_timer_hall.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    h_timer_hall.Init.RepetitionCounter = 0;
    h_timer_hall.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&h_timer_hall);
    HAL_TIM_Base_Start(&h_timer_hall);

    // generate an interrupt every time the timer wraps it's 16 bit limit.
    __HAL_TIM_ENABLE_IT(&h_timer_hall, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TMR4_GLOBAL_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TMR4_GLOBAL_IRQn);

    // enable IRQs for type hall effect sensor GPIOs.
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


//////////////////////////////////////////////////////////////
// optionaly change the wheel diameter fromt he default of 6.5" using inches
void HallInterruptSetWheelDiameterInches(float inches){
    WheelSize_mm = inches * 25.4;
    HallData[0].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
    HallData[1].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
}
//////////////////////////////////////////////////////////////
// optionaly change the wheel diameter fromt he default of 6.5" using mm
void HallInterruptSetWheelDiameterMM(float mm){
    WheelSize_mm = mm;
    HallData[0].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
    HallData[1].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
}

//////////////////////////////////////////////////////////////
// reset the whole structure.
// note that this will then miss the first transition, as it will have no
// 'last' information.
void HallInterruptReset(){
    __disable_irq(); // but we want both values at the same time, without interferance
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    __enable_irq();
    memset((void *)&HallData, 0, sizeof(HallData));
    memset((void *)&local_hall_params, 0, sizeof(local_hall_params));
    #ifdef INVERT_L_DIRECTION
        local_hall_params[LEFT].direction = -1;
    #else
        local_hall_params[LEFT].direction = 1;
    #endif
    #ifdef INVERT_R_DIRECTION
        local_hall_params[RIGHT].direction = 1;
    #else
        local_hall_params[RIGHT].direction = -1;
    #endif

    HallData[0].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
    HallData[1].HallPosnMultiplier = (float)((WheelSize_mm*3.14159265359)/(float)HALL_POSN_PER_REV);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


//////////////////////////////////////////////////////////////
// read values with interupts disabled
// optionally reset posn to zero.
//
void HallInterruptReadPosn( HALL_POSN *p, int Reset ){
    __disable_irq(); // but we want both values at the same time, without interferance
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    __enable_irq();
    for (int i = 0; i < 2; i++){
        p->wheel[i].HallPosn = HallData[i].HallPosn;
        p->wheel[i].HallPosn_mm = HallData[i].HallPosn_mm;
        p->wheel[i].HallSpeed = HallData[i].HallSpeed;
        p->wheel[i].HallSpeed_mm_per_s = HallData[i].HallSpeed_mm_per_s;
        p->wheel[i].HallSkipped = HallData[i].HallSkipped;
        p->wheel[i].HallTimeDiff = HallData[i].HallTimeDiff;

        if (Reset){
            HallData[i].HallPosn = 0;
            HallData[i].HallPosn_mm = 0;
        }
    }
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}



//////////////////////////////////////////////////////////////
// the hall sensors change 1 bit at a time.
// the transtion tells us the direction of movement.
// this table is [last hall value][new hall value], and gives the
// direction of movement.
// table values of 0 represent 'illegal' transitions -
// i.e. if we see a 0 out, then we missed an interrupt
// (never seen in practice yet)
static const int increments[7][7] =
{
    {  0,  0,  0,  0,  0,  0,  0 },
    {  0,  0,  0, -1,  0,  1,  0 },
    {  0,  0,  0,  1,  0,  0, -1 },
    {  0,  1, -1,  0,  0,  0,  0 },
    {  0,  0,  0,  0,  0, -1,  1 },
    {  0, -1,  0,  0,  1,  0,  0 },
    {  0,  0,  1,  0, -1,  0,  0 },
};


TIME_STATS timeStats;

static volatile long long now_us = 0;

long long HallGetuS(){
    unsigned short time = h_timer_hall.Instance->CNT;
    long long timerwraps_copy = timerwraps;
    now_us = ((timerwraps_copy<<16) + time) * 10;
    return now_us;
}

/////////////////////////////////////////////////////////////////////
// called from rising and falling edge interrupts off hall GPIO pins.
// reads all hall sensors, and calculates posn & speed.
void HallInterruptsInterrupt(void){
    // we only want the count from this 100khz clock
    __disable_irq(); // but we want both values at the same time, without interferance
    unsigned long time = h_timer_hall.Instance->CNT;
    long long timerwraps_copy = timerwraps;
    local_hall_params[0].hall = (~(LEFT_HALL_U_PORT->IDR & (LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN))/LEFT_HALL_U_PIN) & 7;
    local_hall_params[1].hall = (~(RIGHT_HALL_U_PORT->IDR & (RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN))/RIGHT_HALL_U_PIN) & 7;

    unsigned short Left = LEFT_HALL_U_PORT->IDR;
    unsigned short Right = RIGHT_HALL_U_PORT->IDR;

    // Get hall sensors values
    // hall_ul = !(Left & LEFT_HALL_U_PIN);
    // hall_vl = !(Left & LEFT_HALL_V_PIN);
    // hall_wl = !(Left & LEFT_HALL_W_PIN);

    // hall_ur = !(Right & RIGHT_HALL_U_PIN);
    // hall_vr = !(Right & RIGHT_HALL_V_PIN);
    // hall_wr = !(Right & RIGHT_HALL_W_PIN);
    __enable_irq();

    // if (local_hall_params[0].last_hall > 0 &&
    //     local_hall_params[0].last_hall != local_hall_params[0].hall){
    //         uint32_t tick = HAL_GetTick();
    //     printf("time:%lu:%lu, left:%d\n",
    //         time,tick,local_hall_params[0].hall);
    // }

    for (int i = 0; i < 2; i++){
        // if this wheel change hall input
        if (local_hall_params[i].last_hall != local_hall_params[i].hall){
            if (local_hall_params[i].last_hall == 0){
                // valid startup condition
            } else {

                local_hall_params[i].zerospeedtimeout = 5; // number of timer wraps to after which to assume speed zero
                local_hall_params[i].hall_time = (timerwraps_copy << 16) | time;
                //local_hall_params[i].hall_time = HallGetuS();
                long long dt = local_hall_params[i].hall_time - local_hall_params[i].last_hall_time;

                // note correction of direction for left wheel
                local_hall_params[i].incr =
                    increments[local_hall_params[i].last_hall][local_hall_params[i].hall] *
                    local_hall_params[i].direction;

                HallData[i].HallPosn = HallData[i].HallPosn + local_hall_params[i].incr;
                HallData[i].HallPosn_mm = (int)((float)HallData[i].HallPosn)*HallData[i].HallPosnMultiplier;
                HallData[i].HallTimeDiff = (unsigned long)dt;

                if (local_hall_params[i].incr != 0){
                    // speed = distance/time
                    // in this case, distance is always 1.
                    // and time is between 10 and 65535
                    HallData[i].HallSpeed = (int)
                        (HALL_SPEED_CALIBRATION/(float)HallData[i].HallTimeDiff) *
                        local_hall_params[i].incr;

                    HallData[i].HallSpeed_mm_per_s = (int)
                        ((float)(HallData[i].HallPosnMultiplier/(float)HallData[i].HallTimeDiff) *
                        (float)local_hall_params[i].incr *
                        (float)HALL_INTERRUPT_TIMER_FREQ);
                } else {
                    // we missed a transition?
                    HallData[i].HallSkipped++;
                }
                // printf("i:%d, time:%lu, posn:%ld\n",i,time,HallData[i].HallPosn);
            }

            // remember for next round
            local_hall_params[i].last_hall = local_hall_params[i].hall;
            local_hall_params[i].last_hall_time = local_hall_params[i].hall_time;
        }
    }
}


/////////////////////////////////////////////////////////////////////
// we keep track of timer wraps for accurate speed measurement.
// and also look for 'stopped'
void TMR4_GLOBAL_IRQHandler(void){
    if (__HAL_TIM_GET_FLAG(&h_timer_hall, TIM_IT_UPDATE) != RESET){
        __HAL_TIM_CLEAR_FLAG(&h_timer_hall, TIM_IT_UPDATE);
        timerwraps++;

     //Test HallInterruptReadPosn
    // HALL_POSN p;
    // HallInterruptReadPosn(&p, 0);
    // printf("left wheel rev %d, rpm %d, pos mm %d, speed mm/s %d timediff %ld\n",
    //     p.wheel[0].HallPosn,p.wheel[0].HallSpeed, p.wheel[0].HallPosn_mm, p.wheel[0].HallSpeed_mm_per_s,
    //      HallData[0].HallTimeDiff);
    // printf("right wheel rev %d, rpm %d, pos mm %d, speed mm/s %d timediff %ld\n",
    //     p.wheel[1].HallPosn,p.wheel[1].HallSpeed, p.wheel[1].HallPosn_mm, p.wheel[1].HallSpeed_mm_per_s,
    //      HallData[1].HallTimeDiff);

        for (int i = 0; i < 2; i++){
            if (local_hall_params[i].zerospeedtimeout <= 0){
                HallData[i].HallSpeed = 0;
                HallData[i].HallSpeed_mm_per_s = 0;
                HallData[i].HallTimeDiff = 0;
            } else {
                local_hall_params[i].zerospeedtimeout--;
            }
        }
    }
}

#endif