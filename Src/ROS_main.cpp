/*
* Implement all ROS functions here.
*/

#include "ros.h"
#include "ROS_subscribe.h"
#include "sensor_msgs/JointState.h"
#include "hallinterrupts.h"


extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000


extern "C" 
{
    void ros_init(void);
    void ros_run(void);
    void get_position(sensor_msgs::JointState *wheelPositions);
    //void HallInterruptReadPosn( HALL_POSN*, int );
}

using namespace ros;

ros::NodeHandle nh; /* ROS node handle */

ros::PublisherType *hovebotState;

void ros_init()
{
    nh.initNode();

    /* Register Publisher  */
    //sensor_msgs::JointState varwheels;
    //wheelPositions = &varwheels;
    //Init wheelPositions

    sensor_msgs::JointState tmpVar;
    hovebotState = nh.addPublisher("hovebot_state",  &tmpVar);

    /* Register Subscriber led */
    rosSubscribeInit(&nh);
}

void ros_run()
{
    sensor_msgs::JointState wheelPositions; //This has to be local variable. for some reason.
    char *names[2] = {"left","right"};
    float position[2], velocity[2], effort[2];

    wheelPositions.name = (char **) &names;
    wheelPositions.name_length = 2;
    wheelPositions.position = (float *) &position;
    wheelPositions.position_length = 2;
    wheelPositions.velocity = (float *) &velocity;
    wheelPositions.velocity_length = 2;
    wheelPositions.effort_length = 2;
    wheelPositions.effort = (float *) &effort;

    nh.spinOnce1();

    //test if we can stop at 60% angle
    // #ifdef CONTROL_MOTOR_TEST
    //   int langle = 60*HALL_POSN_PER_REV/360;
    //   int rangle = -60*HALL_POSN_PER_REV/360;
    //   if(p.wheel[LEFT].HallPosn >= langle) {
    //     pwml = 0;
    //   }
    //   if(p.wheel[RIGHT].HallPosn <=rangle) {
    //     pwmr = 0;
    //   }
    // #endif //CONTROL_MOTOR_TEST

    //Note: If message (wheelPositions) is declared outside function/globally publish1 doesn't work.
    //may be constructor is not getting called.
   // get_position(&wheelPositions);
    nh.publish1(hovebotState->topic_id, &wheelPositions); 
}
 
// void get_position(sensor_msgs::JointState *wheelPositions)
// {
//     HALL_POSN p;
//     HallInterruptReadPosn(&p, 0);
    
//     #ifdef DEBUG_SERIAL_USART3
//     //   uint32_t c_time = nh.getHardware()->time();
//     //   if(pwml != 0) 
//     //   {
//     //     printf("%ld: %ld: left wheel rev %d, rpm %d, skipped %ld\n",c_time, p.wheel[LEFT].HallTimeDiff,
//     //       p.wheel[LEFT].HallPosn,p.wheel[LEFT].HallSpeed,  p.wheel[LEFT].HallSkipped);
//     //   }
//     //   if(pwmr != 0)
//     //   { 
//     //     printf("%ld: %ld: right wheel rev %d, rpm %d, skipped %ld\n",c_time, p.wheel[RIGHT].HallTimeDiff,
//     //       p.wheel[RIGHT].HallPosn,p.wheel[RIGHT].HallSpeed, p.wheel[RIGHT].HallSkipped);
//     //   }
//     #endif //DEBUG_SERIAL_USART3

//     wheelPositions->position[LEFT] = p.wheel[LEFT].HallPosn; 
//     wheelPositions->position[RIGHT] =  p.wheel[RIGHT].HallPosn;

//     wheelPositions->velocity[LEFT] =  p.wheel[LEFT].HallSpeed; 
//     wheelPositions->velocity[RIGHT] =  p.wheel[RIGHT].HallSpeed; 

//     wheelPositions->effort[LEFT] =  pwml; 
//     wheelPositions->effort[RIGHT] =  pwmr; 
    
//     return;
// }