/*
 * ROS_subscribe.h
 *
 *  Created on: Mar 18, 2019
 *      Author:
 */

#ifndef INC_ROS_SUBSCRIBE_H_
#define INC_ROS_SUBSCRIBE_H_

#include "ros.h"
#include "std_msgs/UInt16.h"

uint32_t rosSubscribeInit(ros::NodeHandle *nh);
int16_t getLedState(void);

#endif /* INC_SUBSCRIBE_H_ */