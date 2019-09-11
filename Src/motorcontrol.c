
// ###############################################################################
#include "BLDC_controller.h"            /* Model's header file */
#include "rtwtypes.h"
#include "config.h"
#include "defines.h"

RT_MODEL rtM_Left_;    /* Real-time model */
RT_MODEL rtM_Right_;   /* Real-time model */
RT_MODEL *const rtM_Left = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

P rtP;                           /* Block parameters (auto storage) */

DW rtDW_Left;                    /* Observable states */
ExtU rtU_Left;                   /* External inputs */
ExtY rtY_Left;                   /* External outputs */

DW rtDW_Right;                   /* Observable states */
ExtU rtU_Right;                  /* External inputs */
ExtY rtY_Right;                  /* External outputs */
// ###############################################################################

const uint8_t hall_idx_left  = HALL_IDX_LEFT-1;
const uint8_t hall_idx_right = HALL_IDX_RIGHT-1;

int pwms[2] = {0,0};

volatile WHEEL_POSN_STRUCT wheel_posn[2];

int hall_pin_order[7];

//static const uint16_t pwm_res       = 64000000 / 2 / PWM_FREQ; // = 2000
//Should 64000000 or actually be the SysFreq?
uint16_t pwm_res ; 

const uint16_t hall_cfg_left[6][3]  =
{
	{LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN},
	{LEFT_HALL_U_PIN,LEFT_HALL_W_PIN,LEFT_HALL_V_PIN},
	{LEFT_HALL_V_PIN,LEFT_HALL_U_PIN,LEFT_HALL_W_PIN},
	{LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,LEFT_HALL_U_PIN},
	{LEFT_HALL_W_PIN,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN},
	{LEFT_HALL_W_PIN,LEFT_HALL_V_PIN,LEFT_HALL_U_PIN}
};

const uint16_t hall_cfg_right[6][3] =
{
	{RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN},
	{RIGHT_HALL_U_PIN,RIGHT_HALL_W_PIN,RIGHT_HALL_V_PIN},
	{RIGHT_HALL_V_PIN,RIGHT_HALL_U_PIN,RIGHT_HALL_W_PIN},
	{RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN,RIGHT_HALL_U_PIN},
	{RIGHT_HALL_W_PIN,RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN},
	{RIGHT_HALL_W_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_U_PIN}
};

void motor_counter_reset(uint8_t motor)
{
  wheel_posn[motor].reset_counter = 0;
  wheel_posn[motor].prev_hall = 0;
  wheel_posn[motor].ticks = 0; // 90 per revolution
  wheel_posn[motor].millis_at_tick = 0; //time at last tick
  wheel_posn[motor].millis_at_prev_tick = 0; //time at previous to last tick
  wheel_posn[motor].millis_at_prev_rotation = 0;
}

void motor_counter_increment(uint8_t motor)
{
  wheel_posn[motor].millis_at_prev_tick = wheel_posn[motor].millis_at_tick;
  wheel_posn[motor].millis_at_tick = HAL_GetTick();
  //Find out the direction of motion and #o ticks moved since prev reading
  int posn = hall_pin_order[wheel_posn[motor].hall];
  int prev_posn = hall_pin_order[wheel_posn[motor].prev_hall];
  //Initialize first tick
  if(prev_posn < 0)
  {
    wheel_posn[motor].ticks = 0;
  }
  else //after 1st tick we come here.
  {
    //We assume we won't skip more than 1 tick so we can device direction based on ticks
    if(posn-prev_posn > 0 && posn-prev_posn <= 2)
    {
      wheel_posn[motor].ticks += posn-prev_posn; //because we may move more than 1 tick.
    }
    else if(posn-prev_posn < -2)
    {
      wheel_posn[motor].ticks += 6 + posn-prev_posn;
    }
    else if(posn-prev_posn < 0 && posn-prev_posn >= -2)
    {
      wheel_posn[motor].ticks += posn-prev_posn;  // shoul be reverse direction
    }
    else //posn-prev_posn +ve >= 3
    {
      wheel_posn[motor].ticks += posn-prev_posn - 6;
    }
    //See if we finished a rotation 
    if(abs(wheel_posn[motor].ticks -  wheel_posn[motor].ticks_at_prev_rotation) >= 90)
    {
      wheel_posn[motor].rpm = 1000.0*90.0/
        (abs(wheel_posn[motor].ticks - wheel_posn[motor].ticks_at_prev_rotation )*
        (wheel_posn[motor].millis_at_tick - wheel_posn[motor].millis_at_prev_rotation));
      if(wheel_posn[motor].ticks < 0)
      {
        wheel_posn[motor].rpm *= -1; //reverse direction
      }
      wheel_posn[motor].ticks_at_prev_rotation = wheel_posn[motor].ticks;
      wheel_posn[motor].millis_at_prev_rotation = wheel_posn[motor].millis_at_tick;
    }
  }
  wheel_posn[motor].prev_hall = wheel_posn[motor].hall;

  //DEBUG code only
#ifdef DEBUG
  int rpm = 1000*wheel_posn[motor].rpm;
  printf("%d:%d:rpm=%d:%ld:%lu:%d\n",motor,wheel_posn[motor].hall,rpm,
          wheel_posn[motor].ticks,wheel_posn[motor].millis_at_tick,
          wheel_posn[motor].ticks_at_prev_rotation);
#endif
}

void motor_init()
{
  pwm_res = SystemCoreClock / 2 / PWM_FREQ; // = 2250
  /* Set BLDC controller parameters */  
  rtP.z_ctrlTypSel        = CTRL_TYP_SEL;
  rtP.b_phaAdvEna         = PHASE_ADV_ENA;  
  
  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam  = &rtP;
  rtM_Left->dwork         = &rtDW_Left;
  rtM_Left->inputs        = &rtU_Left;
  rtM_Left->outputs       = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam = &rtP;
  rtM_Right->dwork        = &rtDW_Right;
  rtM_Right->inputs       = &rtU_Right;
  rtM_Right->outputs      = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);

  /* Initialize hall_sensor_positions {1,5,4,6,2,3} to find ticks */
  hall_pin_order[1]=0;
  hall_pin_order[5]=1;
  hall_pin_order[4]=2;
  hall_pin_order[6]=3;
  hall_pin_order[2]=4;
  hall_pin_order[3]=5;
  hall_pin_order[0]=-1; //0 should never come and indicated uninitialized state

  motor_counter_reset(LEFT);
  wheel_posn[LEFT].hall = 0;
  motor_counter_reset(RIGHT);
  wheel_posn[RIGHT].hall = 0;

  #ifndef INVERT_L_DIRECTION
  wheel_posn[LEFT].direction = 1;
  #else
  wheel_posn[LEFT].direction = -1;
  #endif
  #ifndef INVERT_R_DIRECTION
  wheel_posn[RIGHT].direction = 1;
  #else
  wheel_posn[RIGHT].direction = -1;
  #endif

}

void motor_run()
{
  // ############################### MOTOR CONTROL ###############################
 
  int ul, vl, wl;
  int ur, vr, wr;
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & hall_cfg_left[hall_idx_left][0]);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & hall_cfg_left[hall_idx_left][1]);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & hall_cfg_left[hall_idx_left][2]);

    // ----- start tick measurement ---- //
    wheel_posn[LEFT].hall = (~(LEFT_HALL_U_PORT->IDR & (LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN))/LEFT_HALL_U_PIN) & 7;
    wheel_posn[RIGHT].hall = (~(RIGHT_HALL_U_PORT->IDR & (RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN))/RIGHT_HALL_U_PIN) & 7;

    for (int i=0;i<2; i++)
    {
      if(wheel_posn[i].reset_counter) {
        motor_counter_reset(i);
      }
      if(wheel_posn[i].hall != wheel_posn[i].prev_hall) 
      {
        motor_counter_increment(i);
      }
    }
    // ----- end tick measurement ---- //
    /* Set motor inputs here */
    rtU_Left.b_hallA   = hall_ul;
    rtU_Left.b_hallB   = hall_vl;
    rtU_Left.b_hallC   = hall_wl;
    rtU_Left.r_DC      = pwms[LEFT];
    
    /* Step the controller */
    BLDC_controller_step(rtM_Left);

    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;

    /* Apply commands */
    LEFT_TIM->LEFT_TIM_U    = (uint16_t)CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_V    = (uint16_t)CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_W    = (uint16_t)CLAMP(wl + pwm_res / 2, 10, pwm_res-10);
  // =================================================================
  

  // ========================= RIGHT MOTOR ===========================  
    // Get hall sensors values
    uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & hall_cfg_right[hall_idx_right][0]);
    uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & hall_cfg_right[hall_idx_right][1]);
    uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & hall_cfg_right[hall_idx_right][2]);

    /* Set motor inputs here */
    rtU_Right.b_hallA  = hall_ur;
    rtU_Right.b_hallB  = hall_vr;
    rtU_Right.b_hallC  = hall_wr;
    rtU_Right.r_DC     = pwms[RIGHT];

    /* Step the controller */
    BLDC_controller_step(rtM_Right);

    /* Get motor outputs here */
    ur            = rtY_Right.DC_phaA;
    vr            = rtY_Right.DC_phaB;
    wr            = rtY_Right.DC_phaC;
 // motSpeedRight = rtY_Right.n_mot;
 // motAngleRight = rtY_Right.a_elecAngle;

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  // =================================================================
 
 // ###############################################################################
}
