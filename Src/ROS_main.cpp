/*
* Implement all ROS functions here.
*/

#include "ros.h"
#include "ROS_subscribe.h"
#include "std_msgs/ROSString.h"

extern "C" 
{
    void ros_init();
    void ros_run();
}

ros::NodeHandle nh; /* ROS node handle */

ros::PublisherType *chatter;
//extern int16_t ledState;

void ros_init()
{
    /* ROS init, we can start UART hear in not in bsp */ 
    //nh_ = &nh;
    nh.initNode();

    /* Register Publisher Chatter */
    std_msgs::String str_msg;
    chatter = nh.addPublisher("chatter",  &str_msg);

    /* Register Subscriber led */
    rosSubscribeInit(&nh);
}

void ros_run()
{
    nh.spinOnce1();

    //Note: If Msg is declared outside function/globally publish1 doesn't work.
    //may be constructor is not getting called.
    std_msgs::String str_msg;

    char buf[512];
    sprintf(buf,"Millis %ld ledState = %d",HAL_GetTick(), getLedState());
    str_msg.data = buf;
    //printf("%s\n",buf);
    
    //nh.publish1(chatter->topic_id, &str_msg); 

}