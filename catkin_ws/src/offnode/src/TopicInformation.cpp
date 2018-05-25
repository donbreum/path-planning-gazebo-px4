#include "TopicInformation.hpp"

TopicInformation::TopicInformation(){
  ROS_INFO_STREAM("TopicInformation constructor");
  initializeSubscribers();
}

TopicInformation::TopicInformation(ros::NodeHandle* nodehandle)
 : nh(*nodehandle){}

void TopicInformation::initializeSubscribers(){
  ROS_INFO("Initializing subscribers in topic information");
  gps_pos_ = nh_.subscribe<sensor_msgs::NavSatFix>
      ("mavros/global_position/global", 10,&TopicInformation::nav_pos_cb,this);
  gps_heading_ = nh_.subscribe<std_msgs::Float64>
      ("mavros/global_position/compass_hdg", 10,
      &TopicInformation::heading_cb,this);
  alt_pos_ = nh_.subscribe<mavros_msgs::Altitude>
      ("mavros/altitude", 10,&TopicInformation::altitude_cb,this);
  wps_ = nh_.subscribe("mavros/mission/waypoints", 1,
                        &TopicInformation::wps_cb,this);

  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
      ("mavros/setpoint_position/local", 10);

  state_sub_ = nh_.subscribe<mavros_msgs::State>
      ("mavros/state", 10, &TopicInformation::state_cb, this);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
      ("mavros/cmd/arming");

  // tried to use acceleration for the local body with AttitudeTarget
  // but this never worked. Only thrust is working, not pitch, roll nor yaw
  mav_att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>
      ( "mavros/setpoint_raw/attitude", 100 );

  target_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>
      ("mavros/setpoint_velocity/cmd_vel",10);


  // wait for FCU connection
  ros::Rate rate(20.0);
  cout <<"state in initialize topic: " << current_state_.connected << endl;
  while(ros::ok() && !current_state_.connected){
      ros::spinOnce();
      rate.sleep();
  }
}

void TopicInformation::set_mode(){

  offb_set_mode.request.custom_mode = "OFFBOARD";

  if(current_state_.mode != "OFFBOARD")
    if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          ROS_INFO("Offboard enabled");
}

void TopicInformation::initialize_pose(){
  ROS_INFO_STREAM("Pose initializing");
  ros::Rate rate(20.0);
  pose_;
  pose_.pose.position.x = 0;
  pose_.pose.position.y = 0;
  pose_.pose.position.z = 0;

  //send a few setpoints before starting
  for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub_.publish(pose_);
      ros::spinOnce();
      rate.sleep();
  }
}

void TopicInformation::pose(geometry_msgs::PoseStamped pose){
  local_pos_pub_.publish(pose); // den her brugte jeg tidligere
}

void TopicInformation::set_velocity(geometry_msgs::TwistStamped tw){
  target_velocity_.publish(tw);
}

bool TopicInformation::arm(bool arm){

  arm_cmd.request.value = arm;

  if(arming_client_.call(arm_cmd) && arm_cmd.response.success){
    if(arm)
      ROS_INFO("Vehicle armed");
    else
      ROS_INFO("Vehicle disarmed");
  }
}

vector<float> TopicInformation::get_position_data(){
  vector<float> pos_data;
  pos_data.push_back(nav_pos.latitude);
  pos_data.push_back(nav_pos.longitude);
  pos_data.push_back(altitude.local);
  pos_data.push_back(heading.data);
  // cout << "Latitude: " << nav_pos.latitude << endl;
  // cout << "Longitude: " << nav_pos.longitude << endl;
  // cout << "Altitude: " << altitude.local << endl;
  // cout << "Heading: " << heading.data << endl;
  return pos_data;
}

vector<float> TopicInformation::get_current_waypoint(){
  vector<float> tmp;
  return tmp;
}

vector<float> TopicInformation::get_next_waypoint(){
  vector<float> tmp;
  return tmp;
}

int TopicInformation::get_waypoint_index(){
  int waypoint_index = 0;
  for(int k = 0; k < mission_wps.size(); k++){
    if(mission_wps[k].is_current){
      waypoint_index = k;
      return waypoint_index;
    }
  }
  return waypoint_index;
}

std::vector<mavros_msgs::Waypoint> TopicInformation::get_waypoints(){
  cout << "Number of waypoints uploaded: " << mission_wps.size() << endl;
  return mission_wps;
}

void TopicInformation::altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
  altitude = *msg;
}

void TopicInformation::nav_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
  nav_pos = *msg;

}

void TopicInformation::wps_cb(const mavros_msgs::WaypointList::ConstPtr& list){
  mission_wps = list->waypoints;
  ROS_INFO("Waypoints loaded: %i", mission_wps.size());
  for(int k = 0; k < mission_wps.size(); k++){
    ROS_INFO("Waypoint [%i]: Command [%i] Longitude: [%f]"
              " Lattitude: [%f] Altitude: [%f]", k+1,
              mission_wps[k].command, mission_wps[k].y_long,
              mission_wps[k].x_lat, mission_wps[k].z_alt);
  }
}

void TopicInformation::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void TopicInformation::heading_cb(const std_msgs::Float64::ConstPtr& msg){
  heading = *msg;
}
