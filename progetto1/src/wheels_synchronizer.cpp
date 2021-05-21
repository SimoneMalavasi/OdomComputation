#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "progetto1/Motor4Speeds.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


void callback(const robotics_hw1::MotorSpeedConstPtr& msgfr,
              const robotics_hw1::MotorSpeedConstPtr& msgrr,
              const robotics_hw1::MotorSpeedConstPtr& msgfl,
              const robotics_hw1::MotorSpeedConstPtr& msgrl,
              const ros::Publisher& chatter_pub) {
  ROS_INFO ("Received four messages: fr (%u,%f) , rr (%u,%f), fl (%u,%f) and rl (%u,%f)",
            msgfr->header.seq,msgfr->rpm,
            msgrr->header.seq,msgrr->rpm,
            msgfl->header.seq,msgfl->rpm,
            msgrl->header.seq,msgrl->rpm);

    progetto1::Motor4Speeds msg;
    msg.header = msgfl->header;
    msg.rpmfr = msgfr->rpm;
    msg.rpmrr = msgrr->rpm;
    msg.rpmfl = msgfl->rpm;
    msg.rpmrl = msgrl->rpm;
    chatter_pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "/motor_speed_fr", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "/motor_speed_rr", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "/motor_speed_fl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "/motor_speed_rl", 1);
  ros::Publisher chatter_pub = n.advertise<progetto1::Motor4Speeds>("sync_speeds", 1000);

  typedef message_filters::sync_policies
      ::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2,sub3,sub4);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4,chatter_pub));

  ros::spin();

  return 0;
}
