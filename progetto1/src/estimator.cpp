#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/tf.h"



void callback(const geometry_msgs::Vector3StampedConstPtr& msg1,
              const nav_msgs::OdometryConstPtr& msg2,ros::Publisher& pub, ros::Publisher& pub2) {

         if(abs(msg2->twist.twist.angular.z)<=0.01 && abs(msg2->twist.twist.linear.x)>=0.01) {
             float gearRatio{static_cast<float>(msg1->vector.x/msg2->twist.twist.linear.x)};
             geometry_msgs::Vector3 msg;
             msg.x = msg2->twist.twist.linear.x;
             msg.y = msg1->vector.x;
             msg.z = gearRatio;

             pub.publish(msg);
         }

         if(abs(msg2->twist.twist.angular.z)>=0.01 && abs(msg2->twist.twist.angular.x)<=0.01) {
             float baseLine{static_cast<float>((msg1->vector.x - msg1->vector.y)/msg2->twist.twist.angular.z) };
             geometry_msgs::Vector3 msg;
             msg.x = msg2->twist.twist.angular.z;
             msg.y = (msg1->vector.x - msg1->vector.y);
             msg.z = baseLine;

             pub2.publish(msg);
         }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "estimator");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Vector3>("GR_data", 1000);
    ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3> ("BL_data" ,1000);

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "/speed_linear_velocities", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "/scout_odom", 1);

    typedef message_filters::sync_policies
    ::ApproximateTime<geometry_msgs::Vector3Stamped, nav_msgs::Odometry> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2, pub, pub2));

    ros::spin();

    return 0;
}
