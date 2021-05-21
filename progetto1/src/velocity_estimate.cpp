#include "ros/ros.h"
#include "progetto1/Motor4Speeds.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"


class pub_sub {


public:
    pub_sub(){
        sub = n.subscribe("/sync_speeds", 100, &pub_sub::callback, this);
        pub = n.advertise<geometry_msgs::TwistStamped>("/estimated_vel", 100);
        pub2 = n.advertise<geometry_msgs::Vector3Stamped>("/speed_linear_velocities",100);
    }

    void callback(const progetto1::Motor4SpeedsConstPtr& msg){
        float fr = msg->rpmfr/gearRatio;
        float fl = msg->rpmfl/gearRatio;
        float rl = msg->rpmrl/gearRatio;
        float rr = msg->rpmrr/gearRatio;

        float rpmR = (fr+rr)/2;
        float rpmL = (fl+rl)/2;

        float velRight = rpmR*wheelRadius*rpm2rad;
        float velLeft =  -rpmL*wheelRadius*rpm2rad;

        float linVel = (velRight+velLeft)/2;
        float angVel = (velRight-velLeft)/apparentBaseLine;

        wheelsLinSpeeds.header = msg->header;
        wheelsLinSpeeds.vector.x = velRight;
        wheelsLinSpeeds.vector.y = velLeft;
        wheelsLinSpeeds.vector.z = linVel;
        pub2.publish(wheelsLinSpeeds);

        estimatedVel.twist.linear.x = linVel;
        estimatedVel.twist.angular.z = angVel;
        estimatedVel.header = msg->header;
        pub.publish(estimatedVel);


    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub2;
    const float wheelRadius{0.1575} ; // [m]
    const float apparentBaseLine{1.0355} ; // [m]
    const float rpm2rad{2*3.14/60};
    const float gearRatio=38.1631;
    geometry_msgs::TwistStamped estimatedVel;
    geometry_msgs::Vector3Stamped wheelsLinSpeeds;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_node");

    pub_sub my_pub_sub;

    ros::spin();

    return 0;
}
