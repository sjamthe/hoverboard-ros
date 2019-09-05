/*
 * ros_subscribe.cpp
 *	Operate led #2
 *
 *  Created on: Mar 18, 2019
 *      Author: Shirish Jamthe
 * 
 */

#include "ROS_subscribe.h"

void led_cb(unsigned char* cmd_msg);
static int16_t ledState = -1;

void led_cb(unsigned char* msg)
{
	const std_msgs::UInt16 cmd_msg;
	cmd_msg.deserialize(msg);
	
    ledState = cmd_msg.data;
    printf("ledState in CB %d\n",ledState);
    // if(ledState) {
    //     bsp_LedOn(2);
    // }
    // else
    // {
    //     bsp_LedOff(2);
    // }
    
}

int16_t getLedState()
{
    return ledState;
}


uint32_t rosSubscribeInit(ros::NodeHandle *nh)
{
    ros::SubscriberType sub ;
	//nh_ = nh;

	const std_msgs::UInt16 msg;
	sub.topic_name = "led";
	sub.message_type = msg.getType();
	sub.md5sum = msg.getMD5();
	sub.callback = led_cb;
	
	nh->addSubscriber(sub);
	
    return 1;
}
