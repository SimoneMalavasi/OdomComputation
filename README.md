# RoboticsProject

Project for Robotics Course

Authors : Simone Malavasi 10747462
	  Enrico Scomazzon 10525956

Description : 
- wheels_synchronizer.cpp:
Subscribe to the topics /motor_speed_fl /motor_speed_fr /motor_speed_rl /motor_speed_rr with a message filter and synchronizes them. Then it publishes on the topic /sync_speeds a custom message 'Motor4Speeds' which contains 4 synchronized wheels speed with the same time stamp.
	      
- velocity_estimate.cpp:
Subscribes to the topic /sync_speeds and as a callback computes the linear and angular velocities needed for the integration of the kinematic model, and publishes them on the topic /estimated_vel as a TwistedStamped message, this will be used for the odometry estimation. Notice that in this node are present two variables gearRatio and apparentBaseLine,these are estimated parameters and in order to estimate them the /estimated_vel node publishes also onthe topic speed_linear_velocities a Vector3 message, containing the linear speeds of the two wheels and the linear speed v of the robot. These physical quantities will be used in the node /estimator for the parameters mentioned above.

- scout_our_odom.cpp:
Performs the odometry computation. Subscribes to the topic /estimated_vel and integrates the kinematic model of a differential drive robot. The pose of the robot is thus published
on the /our_odom topic as an nav_msgs/Odometry message. Moreover it publishes the custom message /our_custom_odom containing the selected integration method on the topic /scout_our_odom_custom. This node also broadcasts the TF from 'base_link' to 'odom' reference frame and it is the server of the /set_pose and /reset_pose services.

- estimator.cpp:
Subscribes to the topics /speed_linear_velocities and /scout_odom (the odometry by the manifacturer) and gathers data used to perform an offline identication of the parameter gearRatio and apparentBaseLine. The identification approach is described below.


ROS parameters:

 "initial_pose_x"  	 // set initial x pose in odom reference frame
 "initial_pose_y" 	 // set initial y pose in odom reference frame
 "initial_pose_theta""   // set initial theta in odom reference frame

Dynamic reconfiguration:
"integration_method", int_t, 0 if integration method = euler; 1 if integration method = runge-kutta;
It is possible to change the parameter through an enum, which could switch from Euler(0) to Runge_Kutta(1).

TF tree :
			world
			  |
			  |
			 odom
			  |
			  |
		       base-link

Custom Messages:
-'odometryMethod' that contains:
nav_msgs/Odometry odom
std_msgs/String method
and it is published on "\our_custom_odom".

-'Motor4Speeds' that contains:
Header header
float64 rpmfr
float64 rpmrr
float64 rpmfl
float64 rpmrl
and it is published on "\sync_speeds".
  
How to use nodes:
It is possible to launch all the nodes from the launch file through command:
roslaunch progetto1 progetto1.launch

It must be launched first the launch file and then it must be played the bag. In the launch file all the initial parameters are set with respect to the bag1.
In addition, is possible to comment out in the launch file the lines of code relative to the rviz and rqt-reconfigure, to have a complete visual setup. In RVIZ is possible to select the world frame to compare the pose provided by the OptiTrack and odom frame to compare the estimated odometry with the scout_odom.
Optionally, is possible to launch the estimator node to estimate the real gear ratio and the apparent baseline.

Parameters identification: 

- Gear ratio estimation:
In order to estimate the real value of the gear ratio, we used the estimator node. The main idea was to compare the velocity of the odometry provided by the manifacturer with the one estimated by us, setting the nominal gear ratio = 1. We compared them only when the robot was driving straightforward (angular velocity < 0.1 and linear velocity >0.1). We published the values of the velocities on the topic /GR_Data and we print them to a txt file, through the roscommand :
"rostopic echo -p GR_data_gt > GR_data_gt.txt".
This command gathers the data in a matlab-friendly format, thus we used this feature to perform an offline identification on matlab.
The relation between the real linear velocity along x and the one we are estimating is V_manifacturer = V_estimated/gea_ratio (REMARK: V_estimated is being computed with gear_ratio = 1, otherwise we wouldnt obtain the real gear ratio). This relation is linear and the coefficient gear_ratio can be easily estimated through a Least Square fitting approach using the matlab function polyfit() and the obtained gear ratio is 38.1631.
polyfit(X,Y,1), with:
X = vector with V_manifacturer datapoints collected along the bag1 time window
Y = vector with V_estimated datapoints collected along the bag1 time window
1 = degree of the polynomial to fit

- Apparent baseline estimation:
The same procedure of the gear ratio identification has be done to get the apparent baseline. We started setting the gear ratio equal to the one estimated previously. In this case we compared the difference of the linear velocities of the wheels estimated by us with the angular speed provided by the manifacturer, given the relation:
(V_right_estimated-V_left_estimated)/omega_manifacturer = Apparent Baseline
The data are collected only when the robot is purely rotating ((angular velocity > 0.1 and linear velocity < 0.1)). We published the values on the topic /BL_Data and we print them to a txt file. Then the same procedure of the gear ratio estimation has be done, obtaining: apparent baseline = 1.0355 m.
polyfit(X,Y,1), with:
X = vector with omega_manifacturer datapoints collected along the bag1 time window
Y = vector with (V_right_estimated-V_left_estimated) collected datapoints along the bag1 time window
1 = degree of the polynomial to fit
