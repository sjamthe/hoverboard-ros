/*
 * ROS_subscribe.h
 *
 *  Created on: Mar 18, 2019
 *      Author:
 */

#ifndef INC_ROS_SUBSCRIBE_H_
#define INC_ROS_SUBSCRIBE_H_

#include "ros.h"
#include "defines.h"
#include "sensor_msgs/JointState.h"

uint32_t rosSubscribeWheelsCmd(ros::NodeHandle *nh);
sensor_msgs::JointState getWheelPositions(void);

#endif /* INC_SUBSCRIBE_H_ */