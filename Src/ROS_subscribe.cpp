/*
 * ros_subscribe.cpp
 *
 *  Created on: Mar 18, 2019
 *      Author: Shirish Jamthe
 * to test publish use following
 * $ rostopic pub /wheels_cmd sensor_msgs/JointState "{name:['LEFT','RIGHT'], position:[10,10],velocity:[20,20],effort:[60,-60]}"
 */

#include "ROS_subscribe.h"
#include "pid.h"
#include "flashcontent.h"

#define SAFETY_MS 2000
const int position_pwms[2] = {50,55};

extern "C" 
{
	void wheels_pwm_set(void);
}

static sensor_msgs::JointState wheelPositions;
uint32_t wheelInputAt; //MS at which we received the input callback

extern int pwms[2];
extern pid_controller  PositionPid[2];
extern PID_FLOATS PositionPidFloats[2];
extern pid_controller  SpeedPid[2];
extern PID_FLOATS SpeedPidFloats[2];

extern volatile WHEEL_POSN_STRUCT wheel_posn[2];

long ticksTarget[2] = {0,0}; //ticks target set by ROS
float rpmTarget[2] = {0,0}; //velocity/rpm target set by ROS
int motionDirection[2]; //Were we asked to move forward or back?

void wheels_cmd_cb(unsigned char* msg)
{
	wheelInputAt = HAL_GetTick();
	wheelPositions.deserialize(msg); 
  for (int i=0; i < wheelPositions.name_length && i<2; i++)
  {
    ticksTarget[i] = long(wheelPositions.position[i]) + wheel_posn[i].ticks; 
    //
    if(ticksTarget[i]*wheel_posn[i].direction >= wheel_posn[i].ticks*wheel_posn[i].direction)
    {
      motionDirection[i] = 1*wheel_posn[i].direction;
    }
    else
    {
      motionDirection[i] = -1*wheel_posn[i].direction;
    }
    if(wheelPositions.position[i] != 0)
    {
      pid_reset(&PositionPid[i]); //we need to rest integral error for every new target
    }
    //reset speed only if we are not moving
    if(wheel_posn[i].rpm != 0 && wheelPositions.velocity[i] != rpmTarget[i])
    {
      pid_reset(&SpeedPid[i]); 
    }
    rpmTarget[i] = wheelPositions.velocity[i]; //store old rpm target so we don't reset pid
    //printf("%lu: wheels_cmd_cb tickTarget=%ld, dir=%d\n",wheelInputAt,ticksTarget[i],motionDirection[i] );
  }
}

void wheels_pwm_set()
{
  uint32_t now = HAL_GetTick();

  //safety stop if no new update in MS
  if((now - wheelInputAt) > SAFETY_MS)
  {
    for (int i=0; i<2;i++)
    {
      if(pwms[i] != 0) {
        printf("%lu:LIMIT reached. set speed to 0 , %lu, %lu for pwms[%d]=%d ticks=%ld\n",
            now, wheelInputAt, (now - wheelInputAt), i, pwms[i], wheel_posn[i].ticks);
      }
      pwms[i] = 0; //emergency stop motor
	    wheelInputAt = 0; //reset clock;
    }
    return;
  }
  else if (wheelInputAt > 0) //set PWM based on input from ros
  {
    //check wheelPositions.name_length <= 2 for safety
    for (int i=0;i < wheelPositions.name_length && i < 2; i++)
    {
      //If the current position exceed target (with right direction) then stop motor
      if(wheelPositions.position[i] != 0 && 
        ticksTarget[i]*motionDirection[i] <= wheel_posn[i].ticks*motionDirection[i])
      {
        // printf("%lu:Target[%d] met %ld <= %ld pwms=%d\n",now,i,
        //       ticksTarget[i]*motionDirection[i],wheel_posn[i].ticks*motionDirection[i],pwms[i] );
       
        // We met or exceeded the target stop motor
        pwms[i] = 0;
      }
      else if(wheelPositions.position[i] != 0)
      {
        if (pid_need_compute(&PositionPid[i])) {
          // Read process feedback
          PositionPidFloats[i].set = ticksTarget[i];
          PositionPidFloats[i].in = wheel_posn[i].ticks;
          // Compute new PID output value
          pid_compute(&PositionPid[i]);
          //Change actuator value
          int pwm = PositionPidFloats[i].out;
          pwm = CLAMP(pwm, -PWM_LIMIT, PWM_LIMIT);
          pwms[i] = pwm*wheel_posn[i].direction; 

        }
        // printf("%lu:Target[%d] set for %ld <= %ld, pwms=%d\n",now,i,
        //       ticksTarget[i]*motionDirection[i],wheel_posn[i].ticks*motionDirection[i],pwms[i] );
      }
      // rpm velocity 
      else if(wheelPositions.velocity[i] != 0 && wheelPositions.velocity[i] != wheel_posn[i].rpm)
      {
        if (pid_need_compute(&SpeedPid[i])) {
          // Read process feedback
          SpeedPidFloats[i].set = wheelPositions.velocity[i];
          SpeedPidFloats[i].in = wheel_posn[i].rpm;
          // Compute new PID output value
          pid_compute(&SpeedPid[i]);
          //Change actuator value
          int pwm = SpeedPidFloats[i].out;
          pwm = CLAMP(pwm, -PWM_LIMIT, PWM_LIMIT);
          pwms[i] = pwm*wheel_posn[i].direction; 
        }
      }
      //pwm can  be set directly
      else if(wheelPositions.effort[i] != 0)
      {
        //Make sure we are withing prescribed limits
        if(wheelPositions.effort[i] < 0)
        {
          pwms[i] = MAX(-PWM_LIMIT,int(wheelPositions.effort[i])); 
        }
        else
        {
          pwms[i] = MIN(PWM_LIMIT,int(wheelPositions.effort[i])); 
        }
        printf("%lu:New PWM set for pwms[%d]=%d\n",now,i,pwms[i]);
      }     
    } // end of for
  } //end of if-else
} //end of function

uint32_t rosSubscribeWheelsCmd(ros::NodeHandle *nh)
{
    ros::SubscriberType sub ;
    const sensor_msgs::JointState msg;
	sub.topic_name = "wheels_cmd";
	sub.message_type = msg.getType();
	sub.md5sum = msg.getMD5();
	sub.callback = wheels_cmd_cb;
	
	nh->addSubscriber(sub);
	
    return 1;
}
