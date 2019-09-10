/*
 * ros_subscribe.cpp
 *
 *  Created on: Mar 18, 2019
 *      Author: Shirish Jamthe
 * to test publish use following
 * $ rostopic pub /wheels_cmd sensor_msgs/JointState "{name:['LEFT','RIGHT'], position:[10,10],velocity:[20,20],effort:[60,-60]}"
 */

#include "ROS_subscribe.h"

static sensor_msgs::JointState wheelPositions;

void wheels_cmd_cb(unsigned char* msg)
{
	wheelPositions.deserialize(msg);  
}

sensor_msgs::JointState getWheelPositions()
{
	return wheelPositions;
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
