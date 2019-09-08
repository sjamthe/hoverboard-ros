
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

volatile int pwml = 0;
volatile int pwmr = 0;

uint8_t left_hall, prev_left_hall = 0;
uint8_t right_hall, prev_right_hall = 0;
const uint8_t pin_order = {1,5,4,6,2,3};

//static const uint16_t pwm_res       = 64000000 / 2 / PWM_FREQ; // = 2000
//Should 64000000 or actually be the SysFreq?
uint16_t pwm_res       = 72000000 / 2 / PWM_FREQ; // = 2000

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

void motor_init()
{
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
    left_hall = (~(LEFT_HALL_U_PORT->IDR & (LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN))/LEFT_HALL_U_PIN) & 7;
    right_hall = (~(RIGHT_HALL_U_PORT->IDR & (RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN))/RIGHT_HALL_U_PIN) & 7;

    if(left_hall != prev_left_hall) 
    {
      printf("left:%d:right:%d\n",left_hall,right_hall);
      prev_left_hall = left_hall;
    }
    if(right_hall != prev_right_hall) 
    {
      printf("left:%d:right:%d\n",left_hall,right_hall);
      prev_right_hall = right_hall;
    }
    // ----- end tick measurement ---- //
    /* Set motor inputs here */
    rtU_Left.b_hallA   = hall_ul;
    rtU_Left.b_hallB   = hall_vl;
    rtU_Left.b_hallC   = hall_wl;
    rtU_Left.r_DC      = pwml;
    
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
    rtU_Right.r_DC     = pwmr;

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
