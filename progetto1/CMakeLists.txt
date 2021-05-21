cmake_minimum_required(VERSION 3.0.2)
project(progetto1)


find_package(catkin REQUIRED COMPONENTS
  robotics_hw1
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  tf
  tf2 
  tf2_ros dynamic_reconfigure
)

add_service_files(
        FILES
        resetPose.srv
        setPose.srv
        )

add_message_files(
  FILES
    Motor4Speeds.msg
    odometryMethod.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

generate_dynamic_reconfigure_options(
        cfg/parameters.cfg

)

catkin_package(
  CATKIN_DEPENDS robotics_hw1 roscpp rospy std_msgs message_runtime geometry_msgs tf tf2 tf2_ros dynamic_reconfigure
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wheels_synchronizer src/wheels_synchronizer.cpp)
add_executable(velocity_estimate src/velocity_estimate.cpp)
add_executable(estimator src/estimator.cpp)

 target_link_libraries(wheels_synchronizer
   ${catkin_LIBRARIES}
 )
 target_link_libraries(velocity_estimate
        ${catkin_LIBRARIES}
        )
target_link_libraries(estimator
        ${catkin_LIBRARIES}
        )

add_executable(scout_our_odom src/scout_our_odom.cpp)
add_dependencies(scout_our_odom ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(scout_our_odom
        ${catkin_LIBRARIES}
        )


