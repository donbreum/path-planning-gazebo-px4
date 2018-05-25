/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/thread/thread.hpp>
#include <string>
#include "TopicInformation.hpp"
#include "drone_control.hpp"
#include <eigen3/Eigen/Eigen>
#include <mavros_msgs/WaypointList.h> // mavros_msgs::WaypointList
#include <mavros_msgs/Waypoint.h>



#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/TwistStamped.h>


using namespace Eigen;
using namespace std;

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle n;
    //TopicInformation topicinfo(&nh);
    //TopicInformation topicinfo;
    DroneControl drone_control;

    ros::Publisher move_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO_STREAM("setup complete");

    geometry_msgs::TwistStamped move_msg;

    move_msg.twist.linear.x = 0.0;

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 5  ;
    //
    // //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     move_pub.publish(move_msg);
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    while(ros::ok()){

      drone_control.update_drone_position();
      //move_pub.publish(move_msg);
      //local_pos_pub.publish(pose);

      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
