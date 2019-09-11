/*
 * ros_subscribe.cpp
 *
 *  Created on: Mar 18, 2019
 *      Author: Shirish Jamthe
 * to test publish use following
 * $ rostopic pub /wheels_cmd sensor_msgs/JointState "{name:['LEFT','RIGHT'], position:[10,10],velocity:[20,20],effort:[60,-60]}"
 */

#include "ROS_subscribe.h"

extern "C" 
{
	void wheels_pwm_set(void);
}

static sensor_msgs::JointState wheelPositions;
uint32_t wheelPositionsAt;

extern int pwms[2];
extern volatile WHEEL_POSN_STRUCT wheel_posn[2];

long ticksTarget[2] = {0,0}; //ticks target set by ROS
float rpmTarget[2] = {0,0}; //velocity/rpm target set by ROS

void wheels_cmd_cb(unsigned char* msg)
{
	wheelPositionsAt = HAL_GetTick();
	wheelPositions.deserialize(msg);  
	printf("%lu: New wheels_cmd_cb received\n",HAL_GetTick());
}

void wheels_pwm_set()
{
  uint32_t now = HAL_GetTick();

  //safety stop if no new update in 1000ms
  if((now - wheelPositionsAt) > 2000)
  {
    for (int i=0;i<2;i++)
    {
	  if(pwms[i] != 0) {
	  	printf("%lu:LIMIT reached. set speed to 0 , %lu, %lu for pwms[%d]=%d ticks=%ld\n",
	  			now, wheelPositionsAt, (now - wheelPositionsAt), i, pwms[i], wheel_posn[i].ticks);
	  }
      pwms[i] = 0;
      ticksTarget[i] = 0;
      rpmTarget[i] = 0;
	  wheelPositionsAt = 0; //reset clock;
    }
    return;
  }

  //Get wheel positions and set the values
  if(wheelPositions.name_length == 2) 
  {
    for (int i=0;i<2; i++)
    {
      //If position target was set to 0 set a new target, if we met/passed it then reset it
      if(wheelPositions.position[i] != 0)
      {
        if(ticksTarget[i] == 0)
        {
          //Set new target to current + delta requested
          ticksTarget[i] = long(wheelPositions.position[i]*wheel_posn[i].direction) + wheel_posn[i].ticks;
          if(ticksTarget[i] >= wheel_posn[i].ticks)
          {
            // go forward
            pwms[i] = 100*wheel_posn[i].direction;
          }
          else
          {
            // go back
            pwms[i] = -100*wheel_posn[i].direction;
          }
		      printf("%lu:Target set for [%d] %ld=%d+%d\n",now,i,ticksTarget[i],
		  			long(wheelPositions.position[i]), wheel_posn[i].ticks);
        }
        else if((wheel_posn[i].ticks*wheel_posn[i].direction) >= 
                (ticksTarget[i]*wheel_posn[i].direction))
        {
          // We met the target
          printf("%lu:Target met for %d %ld >= %ld\n",now,i,
                wheel_posn[i].ticks, ticksTarget[i]);
          
          ticksTarget[i] = 0;
          pwms[i] = 0;
          wheelPositionsAt = 0; //reset clock;
        }
      }
      // For velocity target we need pwm
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
    }
  }
}

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
