/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include "TopicInformation.hpp"
#include "drone_control.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle n;
    DroneControl drone_control;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ROS_INFO_STREAM("setup complete");
    while(ros::ok()){
      drone_control.update_drone_position();
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
