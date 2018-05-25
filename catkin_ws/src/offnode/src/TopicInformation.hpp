#ifndef TOPICINFORMATION_H
#define TOPICINFORMATION_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h> // mavros_msgs::State
#include <boost/thread/thread.hpp>
#include <string>
#include <sensor_msgs/NavSatFix.h> // sensor_msgs::NavSatFix
#include <mavros_msgs/Altitude.h> // mavros_msgs::Altitude
#include <mavros_msgs/WaypointList.h> // mavros_msgs::WaypointList
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "tf/tf.h"

using namespace std;

class TopicInformation{

public:
  TopicInformation();
  TopicInformation(ros::NodeHandle* nodehandle);
  vector<float> get_position_data();
  std::vector<mavros_msgs::Waypoint> get_waypoints();
  int get_waypoint_index();
  vector<float> get_current_waypoint();
  vector<float> get_next_waypoint();
  void initializeSubscribers();
  bool arm(bool arm);
  void pose(geometry_msgs::PoseStamped pose);
  void set_velocity(geometry_msgs::TwistStamped tw);
  void initialize_pose();
  void set_mode();


  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void nav_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg);
  void wps_cb(const mavros_msgs::WaypointList::ConstPtr& list);
  void heading_cb(const std_msgs::Float64::ConstPtr& msg);

private:
std_msgs::Float64 heading;
ros::NodeHandle nh;
ros::Subscriber gps_pos_;
ros::Subscriber gps_heading_;
ros::Subscriber alt_pos_;
ros::Subscriber wps_;
ros::NodeHandle nh_;
ros::ServiceClient set_mode_client_;
ros::Publisher local_pos_pub_;
ros::Publisher mav_att_pub_;
ros::Publisher target_global_pub_;
ros::Publisher target_velocity_;
geometry_msgs::PoseStamped pose_;
ros::Subscriber state_sub_;
ros::ServiceClient arming_client_;
ros::Time last_request = ros::Time::now();
mavros_msgs::State current_state_;

  ros::Publisher pub_att;
  ros::Publisher pub_thr;

mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;

sensor_msgs::NavSatFix nav_pos;
mavros_msgs::Altitude altitude;
std::vector<mavros_msgs::Waypoint> mission_wps;

};


#endif
