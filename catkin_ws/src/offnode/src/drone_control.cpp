#include "drone_control.hpp"

DroneControl::DroneControl(){
  waypoint_list = tp.get_waypoints();
  // cout << "Waypoints loaded " <<waypoint_list.size() << endl;
  if(waypoint_list.size() == 0)
    ROS_INFO("OBS!! NO WAYPOINTS LOADED");
  else
    create_path_with_splines();

  tp.initialize_pose();
  tp.set_mode();
  tp.arm(true);

  vector<float> position_data = tp.get_position_data();
  float heading = position_data[3];
  initial_heading_rad = angles::from_degrees(heading);

  takeoff_complete = false;
  nav_loiter = false;
  // data_file.open("/home/ubuntu/data_px4.txt");

}
DroneControl::DroneControl(TopicInformation tp){
  //topic_message = tp
}
vector<double> DroneControl::get_target_heading_vector(float bearing){
  double x_vec = add_angles(double(bearing), M_PI/2.0);
  double y_vec = add_angles(double(bearing), M_PI/2.0);
  x_vec = cos(x_vec);
  y_vec = sin(y_vec);

  if(y_vec < 0){
    if(is_north){
      y_vec = abs(y_vec);
    }
  }else
    if(!is_north){
      y_vec = y_vec * -1.0;
    }

  if(x_vec < 0){
    if(is_east)
      x_vec = abs(x_vec);
  }else{
      if(!is_east)
      x_vec = x_vec * -1.0;
    }

  vector<double> target_vector;
  target_vector.push_back(x_vec);
  target_vector.push_back(y_vec);
  return target_vector;
}
float DroneControl::distance_between_two_coords_simple(float lat1, float lon1,
                                                float lat2, float lon2){
  // OBS!! Does not provide provide precise measure
  // float dlat_rad = (lat1 - lat2)*M_PI/180;
  // float dlon_rad = (lon1 - lon2)*M_PI/180;
  // float dlat = lat1 - lat2;
  // float dlon = lon1 - lon2;
  // float dist = sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5;
  // float dist_rad = sqrt((dlat_rad*dlat_rad) + (dlon_rad*dlon_rad)) * 1.113195e5;
  // cout << "dist:" << dist << endl;
  // cout << "dist_rad: " << dist_rad << endl;
  float dummy;
  return dummy;
}
float DroneControl::distance_between_two_coords(float lat1, float lon1,
                                                float lat2, float lon2){
  // return distance between two coordinates
  // might need to be simplified
  // https://www.movable-type.co.uk/scripts/latlong.html

  //cout << std::fixed << std::setprecision(9);

  float dlat_rad = (lat2 - lat1)*M_PI/180.0;
  float dlon_rad = (lon2 - lon1)*M_PI/180.0;
  if( (lon2 - lon1) < 0)
    is_east = false;
  else
    is_east = true;
  if( (lat2 - lat1) < 0)
    is_north = false;
  else
    is_north = true;

  float lat1_rad = lat1*M_PI/180.0;
  float lat2_rad = lat2*M_PI/180.0;

  float a = sin(dlat_rad/2) * sin(dlat_rad/2) +
            cos(lat1_rad) * cos(lat2_rad) * sin(dlon_rad/2) * sin(dlon_rad/2);
  float c = 2 * atan2(sqrt(a),sqrt(1-a));
  float d = 6371e3 * c;  // distance in meters

  return d; // return distance in meters
}
float DroneControl::get_distance_to_current_waypoint(){

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  vector<float> position_data = tp.get_position_data();
  float current_lat = position_data[0];
  float current_long = position_data[1];
  float current_alt = position_data[3];

  float next_lat = waypoint_list[current_waypoint_index+1].x_lat;
  float next_long = waypoint_list[current_waypoint_index+1].y_long;
  float next_alt = waypoint_list[current_waypoint_index+1].z_alt;

  double r =  6371e3;
  double dist_lat = angles::to_degrees(next_lat - current_lat)/360.0;
  dist_lat = dist_lat * r;

  double dist_long = angles::to_degrees((next_long - current_long)/360);
  dist_long = dist_long * r;

  float distance_to_next =
      distance_between_two_coords(current_lat, current_long,
                                  next_lat, next_long);

  return distance_to_next;
}
float DroneControl::add_angles(float angle1, float angle2){

  float angle = angle1 + angle2;
  if(angle > 2.0*M_PI)
    angle -= M_PI;

  else if(angle < -0.0)
      angle += 2.0*M_PI;

  return angle;
}
double DroneControl::get_bearing_to_current_waypoint_simple(){
  // not used yet
  // vector<float> position_data = tp.get_position_data();
  // float current_lat = position_data[0];
  // float current_lon = position_data[1];
  // float target_lat = waypoint_list[current_waypoint_index].x_lat;
  // float target_lon = waypoint_list[current_waypoint_index].y_long;
  //
  // float off_x = target_lon - current_lon;
  // float off_y = target_lat - current_lat;
  // //float bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795;
  // float bearing = M_PI/2.0 + atan2(-off_y, off_x);// * 57.2957795;
  // //cout << "Bearing in simple method: " << bearing << endl;
  // if(bearing < 0)
  //     bearing += 2*M_PI;
      //bearing += 360.00;

  // cout << "Bearing in simple method: " << angles::to_degrees(bearing) << endl;
}
double DroneControl::get_bearing_to_current_waypoint(){

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  vector<float> position_data = tp.get_position_data();
  float current_lat = position_data[0];
  float current_lon = position_data[1];
  float target_lat = waypoint_list[current_waypoint_index].x_lat;
  float target_lon = waypoint_list[current_waypoint_index].y_long;

  current_lat = current_lat * M_PI/180.0;
  target_lat = target_lat * M_PI/180.0;
  float dlon = (target_lon - current_lon) * M_PI/180.0;
  float y = sin(dlon) * cos(target_lat);
  float x = cos(current_lat) * sin(target_lat) -
            sin(current_lat) * cos(target_lat) * cos(dlon);
  float angle = atan2(y, x);
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  angle = fmod(((angle * 180/M_PI) + 360), 360);
  return angle_rad; // return in radians
  //return angle; // return angle in degrees
}
double DroneControl::get_bearing_between_two_waypoints(float current_lat,
                                                       float current_lon,
                                                       float target_lat,
                                                       float target_lon){

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  current_lat = current_lat * M_PI/180.0;
  target_lat = target_lat * M_PI/180.0;
  float dlon = (target_lon - current_lon) * M_PI/180.0;
  float y = sin(dlon) * cos(target_lat);
  float x = cos(current_lat) * sin(target_lat) -
            sin(current_lat) * cos(target_lat) * cos(dlon);
  float angle = atan2(y, x);
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  angle = fmod(((angle * 180.0/M_PI) + 360), 360);

  return angle_rad; // return in radians
  //return angle; // return angle in degrees
}
void DroneControl::calculate_velocity_angular(float &bearing, float &heading){

  double shortest_dist = angles::shortest_angular_distance(
    heading, bearing);
  double shortest_dist_abs = abs(shortest_dist);
  double err = bearing - heading;

  if(shortest_dist < 0)
    err = -err;

  double z_angular_velocity = pid_heading.calculate(0, shortest_dist);
  body_velocity.angular_z = z_angular_velocity;
}
void DroneControl::calculate_velocity_vertical(float &target_alt, float &cur_alt){
  double vel_z = pid_height.calculate(target_alt, cur_alt);
  body_velocity.linear_z = vel_z;
}
void DroneControl::calculate_velocity_heading(float &bearing, float &bearing_pos_to_wp, float &cross_track_err){
  double heading_x = cos(add_angles(double(-heading), M_PI/2.0));
  double heading_y = sin(add_angles(double(-heading), M_PI/2.0));
  double bearing_to_wp_x = cos(add_angles(double(-bearing_pos_to_wp), M_PI/2.0));
  double bearing_to_wp_y = sin(add_angles(double(-bearing_pos_to_wp), M_PI/2.0));
  double vel_x = heading_x * ground_speed;  
  double vel_y = heading_y * ground_speed; 

  double diff_bearing = bearing - bearing_pos_to_wp;

  double lateral_vel_x;
  double lateral_vel_y;
  if(diff_bearing < 0){
    lateral_vel_x = heading_y;
    lateral_vel_y = -heading_x;
  }
  else{
    lateral_vel_x = -heading_y;
    lateral_vel_y = heading_x;
  }

  double lateral_err = abs(pid_lat_cmd.calculate(0, cross_track_err));
  
  vel_x = vel_x + lateral_vel_x*lateral_err;
  vel_y = vel_y + lateral_vel_y*lateral_err;

  body_velocity.linear_x = vel_x;
  body_velocity.linear_y = vel_y;
}
void DroneControl::set_velocity_body(){

  move_msg.twist.linear.z = body_velocity.linear_z;
  // to make realistic, the drone will start moving forward
  // as soon the altitude is a few cm above ground
  if(cur_altitude > 0.1){
    move_msg.twist.angular.z = body_velocity.angular_z;
    move_msg.twist.linear.x = body_velocity.linear_x;
    move_msg.twist.linear.y = body_velocity.linear_y;
  }

  tp.set_velocity(move_msg);
}
void DroneControl::calculate_velocity_body(float &bearing,
                                           float &bearing_pos_to_wp,
                                           float &cross_track_err){

  calculate_velocity_angular(bearing, heading);
  calculate_velocity_vertical(target_altitude, cur_altitude);
  calculate_velocity_heading(bearing, bearing_pos_to_wp, cross_track_err);

}
#pragma region cross_track_err
float DroneControl::calculate_cross_track_error(float &bearing_target,
                                                float &distance_to_wp,
                                                float &bearing_wp_to_wp){

float theta = bearing_wp_to_wp - bearing_target;
float theta_deg = angles::from_degrees(theta);
if(theta_deg > 270)
  theta += 2.0*M_PI;
else if(theta_deg > 90 and theta_deg < 270)
  cout << "OBS OBS!! THERE IS SOME ERROR HERE" << endl;
theta = abs(theta);
float cross_track_error = distance_to_wp * sin(theta);
//cout << "cross_track_error: " << cross_track_error << endl;

return cross_track_error;
}
#pragma endregion cross_track_err

vector<LoiterCoordinates>  DroneControl::get_loiter_target_coordinates(float lat0, float lon0, float radius){
  vector<LoiterCoordinates> coordinates;
  LoiterCoordinates loiter_coord;
  loiter_coord.lat = lat0;
  loiter_coord.lon = lon0;
  // top circle
  loiter_coord.lat = lat0 + (180/M_PI) * (radius/6378137);
  loiter_coord.lon = lon0;

  coordinates.push_back(loiter_coord);

  // right point circle
  loiter_coord.lat = lat0;
  loiter_coord.lon = lon0 + (180.0/M_PI) * (radius/6378137) / cos(M_PI/180.0*lat0);

  coordinates.push_back(loiter_coord);

  // bottom point circle
  loiter_coord.lat = lat0 + (180.0/M_PI) * (-radius/6378137);
  loiter_coord.lon = lon0;

  coordinates.push_back(loiter_coord);

  // left point circle
  loiter_coord.lat = lat0;
  loiter_coord.lon = lon0 + (180/M_PI) * (-radius/6378137) / cos(M_PI/180.0*lat0);

  coordinates.push_back(loiter_coord);

  return coordinates;
}
int DroneControl::get_closest_loiter_coordinate(){
  int chosen_index = 0;
  double min_distance = 9999999.0;
  for(int i = 0; i < loiter_coordinates.size(); i++){
    double loiter_lat = loiter_coordinates[i].lat;
    double loiter_lon = loiter_coordinates[i].lon;

    double distance = distance_between_two_coords(cur_pos_lat, cur_pos_lon, loiter_lat, loiter_lon);
    if(distance < min_distance){
      chosen_index = i;
      min_distance = distance;
    }
  }
  return chosen_index;
}
void DroneControl::update_drone_position(){
  float dist_to_next_wp = get_distance_to_current_waypoint();
  // cout << "DISTANCE TO NEXT WP: " << dist_to_next_wp << endl;
  ground_speed_target = 5;
  if(is_next_wp_loiter_wp(current_waypoint_index))
    if(dist_to_next_wp < threshold_distance_to_loiter_waypoint && !nav_loiter)
      current_waypoint_index++;
  
  // go down to prepare for landing
  // cout << "NEXT CMD: " << waypoint_list[current_waypoint_index+1].command << endl;
  if(waypoint_list[current_waypoint_index+1].command == MAV_CMD_NAV_LAND)
    if(dist_to_next_wp < THRESHOLD_DISTANCE_LANDING_WP_APPROACH){
      waypoint_list[current_waypoint_index].z_alt = 3;
      ground_speed_target = MAX_GROUND_SPEED/2;
    }
  
  if(dist_to_next_wp < threshold_distance_to_waypoint && !nav_loiter)
    current_waypoint_index++;

  update_drone_data();
  // check if drone is passed the line segment, which it is tracking
  if(is_beyond_line_segment(previous_wp_lat, previous_wp_lon, next_wp_lat, next_wp_lon) && !nav_loiter)
    current_waypoint_index++;

  float bearing = 0;
  float bearing_pos_to_wp = 0;
  float cross_track_error = 0;
  float wp0_lat = 0;
  float wp0_lon = 0;
  float wp1_lat = 0;
  float wp1_lon = 0;
  int MAV_CMD =  waypoint_list[current_waypoint_index].command;
  // cout << "current command: " << MAV_CMD<< endl;
  // cout << "current waypoint: " << current_waypoint_index+1 << endl;
  switch(MAV_CMD){
    case MAV_CMD_NAV_TAKEOFF:{
      if(cur_altitude > 0.5){
        wp0_lat = previous_wp_lat;
        wp0_lon = previous_wp_lon;
        wp1_lat = next_wp_lat;
        wp1_lon = next_wp_lon;
        ground_speed = ground_speed_target;
      }
      break;
    }
    case MAV_CMD_NAV_WAYPOINT:{
        wp0_lat = previous_wp_lat;
        wp0_lon = previous_wp_lon;
        wp1_lat = next_wp_lat;
        wp1_lon = next_wp_lon;
        ground_speed = ground_speed_target;
      break;
    }
    case MAV_CMD_NAV_LOITER_TIME:{
      // initialize first time it enters at every loiter waypoint
      if(!nav_loiter)
        init_loiter();
      update_loiter_coords(wp0_lat, wp0_lon, wp1_lat, wp1_lon, dist_to_next_wp);
      double duration = ((clock() - start_time_loiter) / double(CLOCKS_PER_SEC)*10.0);
      // loiter is done if duration is larger than the set loiter time
      if(duration > loiter_param1){
        nav_loiter = false;
        // fly towards next waypoint by setting current wp as wp command.
        waypoint_list[current_waypoint_index].command = MAV_CMD_NAV_WAYPOINT;
      }
      ground_speed = ground_speed_target;
      break;
    }
    case MAV_CMD_NAV_LOITER_TURNS:{ // not tested, not supported by QGC
      if(!nav_loiter)
        init_loiter();
      update_loiter_coords(wp0_lat, wp0_lon, wp1_lat, wp1_lon, dist_to_next_wp);

      if(loiter_turns > loiter_param1){
        nav_loiter = false;
        waypoint_list[current_waypoint_index].command = MAV_CMD_NAV_WAYPOINT;
      }
      break;
    }
    case MAV_CMD_NAV_LAND:{
      static float fixed_heading = heading;
      if(!land_initiated)
        fixed_heading = heading;

      target_altitude = 0;
      if(cur_altitude < 0.5)
        ground_speed = 0;
      else
        ground_speed = 1;
      if(cur_altitude < 0.1){
        tp.arm(false);
        data_file.close();
      }
      land_initiated = true;
      bearing = fixed_heading;
      bearing_pos_to_wp = fixed_heading;
      cross_track_error = 0;
      break;
    }
    default:{
      cout << "NO VALID COMMAND FROM MAVLINK - LAND AT LOCATION" << endl;
      target_altitude = 0;
      ground_speed = 0;
      if(cur_altitude < 0.1)
        tp.arm(false);
      break;
      }
  }

  if(!land_initiated){
    bearing = get_bearing_between_two_waypoints(wp0_lat,
                                                wp0_lon,
                                                wp1_lat,
                                                wp1_lon);
    bearing_pos_to_wp = get_bearing_between_two_waypoints(cur_pos_lat,
                                                          cur_pos_lon,
                                                          wp1_lat,
                                                          wp1_lon);
    cross_track_error = calculate_cross_track_error(bearing_pos_to_wp,
                                                    dist_to_next_wp,
                                                    bearing);
  }

  calculate_velocity_body(bearing, bearing_pos_to_wp, cross_track_error);
  set_velocity_body();
  // data_file << clock() << ";" 
  //           << current_waypoint_index << ";" 
  //           << cross_track_error << ";" 
  //           << target_altitude << ";" 
  //           << cur_altitude << endl;
  // cout << endl << "------------------" << endl;

}
void DroneControl::update_drone_data(){
  vector<float> position_data = tp.get_position_data();
  cur_pos_lat = position_data[0];
  cur_pos_lon = position_data[1];
  previous_wp_lat = waypoint_list[current_waypoint_index].x_lat;
  previous_wp_lon = waypoint_list[current_waypoint_index].y_long;
  target_altitude = waypoint_list[current_waypoint_index].z_alt;
  next_wp_lat = waypoint_list[current_waypoint_index+1].x_lat;
  next_wp_lon = waypoint_list[current_waypoint_index+1].y_long;
  heading = angles::from_degrees(position_data[3]);
  cur_altitude = position_data[2]; 
}
bool DroneControl::is_beyond_line_segment(float x1, float y1, float x2, float y2){
    // method from paparazzi
    // https://goo.gl/xA5bHx
    float px = cur_pos_lat;
    float py = cur_pos_lon;

    px = px - x1;
    py = py - y1;

    float zx = x2 - x1;
    float zy = y2 - y1;

    float beta = atan2f(zy, zx);
    float cosb = cosf(-beta);
    float sinb = sinf(-beta);
    float zxr = zx * cosb - zy * sinb;
    float pxr = px * cosb - py * sinb;

    if((zxr > 0 && pxr > zxr) || (zxr < 0 && pxr < zxr)) {
        return true;
    }
    return false;
}
bool DroneControl::is_next_wp_loiter_wp(int current_wp_index){
  bool is_loiter = false;
  if(waypoint_list[current_wp_index+1].command == MAV_CMD_NAV_LOITER_UNLIM ||
     waypoint_list[current_wp_index+1].command == MAV_CMD_NAV_LOITER_TURNS ||
     waypoint_list[current_wp_index+1].command == MAV_CMD_NAV_LOITER_TIME){

       is_loiter = true;
    }
  return is_loiter;
}
void DroneControl::create_path_with_splines(){
// c++ implementation of cubic hermite interpolation
// modified to load wp from drone
// https://blog.demofox.org/2015/08/08/cubic-hermite-interpolation/

  // ofstream file;
  // file.open("/home/per/coords.txt");
  // file << std::fixed << std::setprecision(9);
  cout << std::fixed << std::setprecision(9);

  int num_of_sample_points =  waypoint_list.size()*NUMBERS_OF_INTERPOLATION_POINTS;
  const float c_numSamples = num_of_sample_points;
  std::vector<mavros_msgs::Waypoint> waypoint_list_splines(num_of_sample_points);

  TPointList2D points(waypoint_list.size());

  int idx = 0;
  for(auto wp : waypoint_list){
    points[idx][0] = wp.x_lat;
    points[idx][1] = wp.y_long;
    //cout << wp.x_lat << "," << wp.y_long << endl;
    idx++;
  }

  int j = 0;
  for (int i = 0; i < c_numSamples; ++i)
  {
    float percent = ((float)i) / (float(c_numSamples - 1));
    float x = 0.0f;
    float y = 0.0f;

    float tx = (points.size() -1) * percent;
    int index = int(tx);
    float t = tx - floor(tx);

    std::array<float, 2> A = get_index_clamped(points, index - 1);
    std::array<float, 2> B = get_index_clamped(points, index + 0);
    std::array<float, 2> C = get_index_clamped(points, index + 1);
    std::array<float, 2> D = get_index_clamped(points, index + 2);
    x = cubic_hermite(A[0], B[0], C[0], D[0], t);
    y = cubic_hermite(A[1], B[1], C[1], D[1], t);
    
    // file << x <<"," << y << ", 0" << endl;
    waypoint_list_splines[i].x_lat = x;
    waypoint_list_splines[i].y_long = y; 
    waypoint_list_splines[i].z_alt = waypoint_list[j].z_alt; 
    waypoint_list_splines[i].param1 = waypoint_list[j].param1;
    waypoint_list_splines[i].command = waypoint_list[j].command;
    if(i % NUMBERS_OF_INTERPOLATION_POINTS == 0)
      if(!(i==0))
        j++;
  }
  // file.close();
  // printf("\n");

  waypoint_list = waypoint_list_splines;

}
float DroneControl::cubic_hermite(float A, float B, float C, float D, float t){
    float a = -A/2.0f + (3.0f*B)/2.0f - (3.0f*C)/2.0f + D/2.0f;
    float b = A - (5.0f*B)/2.0f + 2.0f*C - D / 2.0f;
    float c = -A/2.0f + C/2.0f;
    float d = B;
  
    return a*t*t*t + b*t*t + c*t + d;
}
void DroneControl::init_loiter(){
  // get loiter time from waypoint
  vector<float> position_data = tp.get_position_data();
  loiter_param1 = waypoint_list[current_waypoint_index].param1; // needs to be checked
  // set time ref
  start_time_loiter = clock();
  nav_loiter = true;
  // get loiter coordinates
  loiter_coordinates = get_loiter_target_coordinates(previous_wp_lat, previous_wp_lon, LOITER_RADIUS);

  // get first index/point for loiter
  // obtain point closest to uav
  current_loiter_index = get_closest_loiter_coordinate();
  loiter_turns = 0;
}
void DroneControl::update_loiter_coords(float &wp0_lat, float &wp0_lon, float &wp1_lat, float &wp1_lon, float &dist_2_wp){
  float prev_loiter_lat = loiter_coordinates[current_loiter_index].lat;
  float prev_loiter_lon = loiter_coordinates[current_loiter_index].lon;

  int next_index = current_loiter_index + 1;
  if(current_loiter_index == loiter_coordinates.size() - 1)
    next_index = 0;

  float next_loiter_lat = loiter_coordinates[next_index].lat;
  float next_loiter_lon = loiter_coordinates[next_index].lon;
  double dist_to_loiter_pt = distance_between_two_coords(cur_pos_lat, cur_pos_lon, next_loiter_lat, next_loiter_lon);

  wp0_lat = prev_loiter_lat;
  wp0_lon = prev_loiter_lon;
  wp1_lat = next_loiter_lat;
  wp1_lon = next_loiter_lon;
  dist_2_wp = dist_to_loiter_pt;

  if(dist_to_loiter_pt < threshold_distance_to_waypoint || 
    is_beyond_line_segment(prev_loiter_lat, prev_loiter_lon, next_loiter_lat, next_loiter_lon)){
    if(current_loiter_index == loiter_coordinates.size()-1){
      current_loiter_index = 0;
      loiter_turns++;
    }else{
      current_loiter_index++;
      loiter_turns++;
    }
  }   
}

