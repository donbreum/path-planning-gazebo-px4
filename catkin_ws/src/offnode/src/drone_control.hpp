#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <mavros_msgs/WaypointList.h> // mavros_msgs::WaypointList
#include "TopicInformation.hpp"
#include "tf/tf.h"
#include "angles/angles.h"
#include "pid.h"
#include <ctime>
#include <numeric>
#include <limits>

#define MAV_CMD_NAV_WAYPOINT 16
#define MAV_CMD_NAV_LOITER_UNLIM 17
#define MAV_CMD_NAV_LOITER_TURNS 18
#define MAV_CMD_NAV_LOITER_TIME 19
#define MAV_CMD_NAV_RETURN_TO_LAUNCH 20
#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_NAV_TAKEOFF 22
#define MAV_CMD_NAV_LAND_LOCAL 23
#define MAV_CMD_NAV_TAKEOFF_LOCAL 24

#define LOITER_TIME_SECONDS 15
#define LOITER_RADIUS 30

// when a new waypoint is loaded, distance in meters
#define threshold_distance_to_waypoint 3
#define threshold_distance_to_loiter_waypoint 35 // should be approx double radius distance
class TopicInformation;
using namespace std;

struct LoiterCoordinates{
  double lat;
  double lon;
};

struct BodyVelocity{
  double linear_x;
  double linear_y;
  double linear_z;
  double angular_z;
};

class DroneControl{

public:

  DroneControl();
  DroneControl(TopicInformation tp);
  void update_drone_data();
  void update_drone_position();
  float distance_between_two_coords(float lat1, float lon1, float lat2,
                                    float lon2);
  float distance_between_two_coords_simple(float lat1, float lon1, float lat2,
                                    float lon2);
  float get_distance_to_current_waypoint();
  double get_bearing_to_current_waypoint();
  double get_bearing_to_current_waypoint_simple();
  float calculate_cross_track_error(float bearing_target, float distance_to_wp,
                                    float bearing_wp_to_wp);
  double get_bearing_between_two_waypoints(float current_lat, float current_lon,
                                           float target_lat, float target_lon);
  float add_angles(float a1, float a2);
  void calculate_velocity_body(float bearing, float bearing_pos_to_wp, float cross_track_err);
  void calculate_velocity_angular(float bearing, float heading);
  void calculate_velocity_vertical(float target_height, float height);
  void set_velocity_body();
  vector<double> get_target_heading_vector(float bearing);
  vector<LoiterCoordinates> get_loiter_target_coordinates(float lat, float lon, float radius);
  int get_closest_loiter_coordinate();
  bool is_beyond_line_segment(float x1, float y1, float x2, float y2);
  bool is_next_wp_loiter_wp(int current_wp_index);
  void init_loiter();
  void update_loiter_coords(float &wp0_lat, float &wp0_lon, float &wp1_lat, float &wp1_lon, float &dist_2_wp);

private:

  TopicInformation tp;
  std::vector<mavros_msgs::Waypoint> waypoint_list;
  geometry_msgs::TwistStamped move_msg;
  int current_waypoint_index = 0;
  double initial_heading_rad = M_PI/2;
  BodyVelocity body_velocity;
  vector<LoiterCoordinates> loiter_coordinates;
  PID pid_height = PID(0.1, 0.5, -0.5, 1.0, 0.0, 0.0);
  //PID pid_heading = PID(0.1, 0.5, -0.5, 0.05, 0.05, 0.0001);
  PID pid_heading = PID(0.1, 0.5, -0.5, 1.0, 0.1, 0.01);
  PID pid_lat_cmd = PID(0.01, 3.0, -3.0, 0.3, 0.1, 0.01);
  bool is_north;
  bool is_east;

  bool takeoff_complete;
  bool nav_loiter;
  float loiter_wp_time;
  float loiter_radius;
  int current_loiter_index;
  clock_t start_time_loiter;

  float target_altitude;

  float cur_pos_lat;
  float cur_pos_lon;
  float previous_wp_lat;
  float previous_wp_lon;
  float next_wp_lat;
  float next_wp_lon;
  float heading;
  float cur_altitude;
};
#endif
