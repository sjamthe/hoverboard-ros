/*
* Implement all ROS functions here.
*/

#include "ros.h"
#include "ROS_subscribe.h"
#include "std_msgs/ROSString.h"
#include "sensor_msgs/JointState.h"
#include "hallinterrupts.h"
#include "defines.h"

extern volatile WHEEL_POSN_STRUCT wheel_posn[2];
extern int pwms[2];

uint32_t publish_time = 0;
extern "C" 
{
    void ros_init(void);
    void ros_run(void);
    void publish_hovebot_state(void);
    void wheels_pwm_set(void);
}

using namespace ros;

ros::NodeHandle nh; /* ROS node handle */

ros::PublisherType *hovebotState;
ros::PublisherType *chatter;


void ros_init()
{
    nh.initNode();
    //nh.setSpinTimeout(5); //timeout after 5ms

    /* Register Publisher hovebot_state  */
    sensor_msgs::JointState tmpVar;
    hovebotState = nh.addPublisher("hoverbot_state",  &tmpVar);

    /* Register Publisher chatter  */
    std_msgs::String tmpStr;
    chatter = nh.addPublisher("chatter",  &tmpStr);

    /* Register Subscriber */
    rosSubscribeWheelsCmd(&nh);
}

void publish_hovebot_state(void)
{
    sensor_msgs::JointState wheelPositions; //This has to be local variable. for some reason.
    char *names[2] = {"LEFT","RIGHT"};
    float position[2] = {wheel_posn[LEFT].ticks,wheel_posn[RIGHT].ticks};
    float velocity[2] = {wheel_posn[LEFT].rpm, wheel_posn[RIGHT].rpm};
    float effort[2] = {pwms[LEFT], pwms[RIGHT]};

    wheelPositions.name = (char **) &names;
    wheelPositions.name_length = 2;
    wheelPositions.position = (float *) &position;
    wheelPositions.position_length = 2;
    wheelPositions.velocity = (float *) &velocity;
    wheelPositions.velocity_length = 2;
    wheelPositions.effort_length = 2;
    wheelPositions.effort = (float *) &effort;
    wheelPositions.header.stamp = nh.now();

    //Note: If message (wheelPositions) is declared outside function/globally publish1 doesn't work.
    //may be constructor is not getting called.
    int retval = nh.publish1(hovebotState->topic_id, &wheelPositions); 
    if(retval < 0)
    {
        printf("hoverbot_state publish1 failed %d\n",retval);
    }

#ifdef DEBUG
    char buf[512];
    std_msgs::String tmpStr;
    tmpStr.data = buf;
    int motor=0;
    int rpm = 1000*wheel_posn[motor].rpm;
    // sprintf(buf,"%d:%d:rpm=%d:%ld:%lu:%d\n",motor,wheel_posn[motor].hall,rpm,
    //     wheel_posn[motor].ticks,wheel_posn[motor].millis_at_tick,
    //     wheel_posn[motor].ticks_at_prev_rotation);
    sprintf(buf,"%d: Hello world\n",publish_time);
    //int retval2 = nh.publish1(chatter->topic_id, &tmpStr);
#endif
}

void ros_run()
{
    uint32_t now = HAL_GetTick();

    if(nh.spinOnce1() != 0)
    {
        printf("%lu:ERROR: spinOnce1 returned error probably SPIN_TIMEOUT\n",now);
        return;
    }
    //Publish every 100ms (10Hz)
    if ((now - publish_time) > 100) {
        //printf("publishing %ld, %ld\n",now, (now - publish_time));
        publish_hovebot_state();
        publish_time = now;
    }
    wheels_pwm_set();
}