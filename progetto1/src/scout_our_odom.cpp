#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <progetto1/parametersConfig.h>
#include <progetto1/odometryMethod.h>
#include "progetto1/resetPose.h"
#include "progetto1/setPose.h"


class Odometry {

public:

    Odometry() {
        sub = n.subscribe("/estimated_vel", 100, &Odometry::callback, this);
        pub = n.advertise<nav_msgs::Odometry>("/our_odom", 100);
        resetService = n.advertiseService("reset_pose", &Odometry::reset, this);
        setService = n.advertiseService("set_pose", &Odometry::set, this);
        pubCustom = n.advertise<progetto1::odometryMethod>("/scout_our_odom_custom", 100);
        n.getParam("/initial_pose_x",x);
        n.getParam("/initial_pose_y",y);
        n.getParam("/initial_pose_theta",theta);
        f = boost::bind(&Odometry::callbackParam,this, _1);
        server.setCallback(f);

    }


    void callback(const geometry_msgs::TwistStampedConstPtr& msg){

        double Ts{msg->header.stamp.toSec()-odometry.header.stamp.toSec()};
        //ROS_INFO("Ts! %f  ", Ts);
        odometry.header.stamp = msg->header.stamp;

        float omega = msg->twist.angular.z;
        float v = msg->twist.linear.x;

        if(integrationMethod == 0){
            //ROS_INFO("Using Euler! %i is the param ", integrationMethod);
            x = x + v*Ts*cos(theta);
            y = y + v*Ts*sin(theta);
            theta = theta + omega*Ts;
        } else if(integrationMethod == 1){
            //ROS_INFO("Using Rung Kutta! %i is the param ", integrationMethod);
            x = x + v*Ts*cos(theta + (omega*Ts)/2);
            y = y + v*Ts*sin(theta + (omega*Ts)/2);
            theta = theta + omega*Ts;
        }

        tf::Quaternion q{pose2Quaternion(x,y,theta)};

        odometry.header = msg->header;
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_link";

        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0;

        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();

        odometry.twist.twist = msg->twist;
        pub.publish(odometry);

        transform.setOrigin( tf::Vector3(x, y, 0) );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        progetto1::odometryMethod customMsg;
        customMsg.odom = odometry;
        if(integrationMethod==0){
            customMsg.method.data = "euler";
        } else if (integrationMethod == 1) {
            customMsg.method.data = "rk";
        }
        pubCustom.publish(customMsg);
    }


private:
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pubCustom;
    ros::ServiceServer resetService;
    ros::ServiceServer setService;
    dynamic_reconfigure::Server<progetto1::parametersConfig> server;
    dynamic_reconfigure::Server<progetto1::parametersConfig>::CallbackType f;
    nav_msgs::Odometry odometry;
    float x{} ; // [m]
    float y{} ; // [m]
    float theta{};
    int integrationMethod;  //true = euler  ,   false = runge kutta

    tf::Quaternion pose2Quaternion(const float x,const float y,const float theta) {
        transform.setOrigin( tf::Vector3(x, y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        return q;
    }

    bool set(progetto1::setPose::Request  &req,
             progetto1::setPose::Response &res)
    {
            x = req.x_in;
            y = req.y_in;
            theta = req.theta_in;

        return true;
    }

    bool reset(progetto1::resetPose::Request  &req,
               progetto1::resetPose::Response &res)
    {
        x = 0;
        y = 0;

        return true;
    }

    void callbackParam(progetto1::parametersConfig &config) {
        ROS_INFO("Reconfigure Request:  %s ",
                 config.integration_method == 0?"Euler integration":"Runge Kutta integration");  //true: euler, false: runge kutta

        integrationMethod = config.integration_method;

    }


};



int main(int argc, char **argv) {
    ros::init(argc, argv, "scout_our_odom");
    Odometry myOdometry;


    ros::spin();
    return 0;
}
