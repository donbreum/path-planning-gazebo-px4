#include "drone_control.hpp"

DroneControl::DroneControl(){
  cout << "constructor i dronecontrol "<< endl;
  waypoint_list = tp.get_waypoints();
  cout << "waypoints loaded in dronecontrol: " <<waypoint_list.size() << endl;
  if(waypoint_list.size() == 0)
    ROS_INFO("OBS!! NO WAYPOINTS LOADED");

  tp.initialize_pose();
  tp.set_mode();
  tp.arm(true);

  vector<float> position_data = tp.get_position_data();
  float heading = position_data[3];
  initial_heading_rad = angles::from_degrees(heading);
  cout << "initial heading: " << initial_heading_rad << endl;

  takeoff_complete = false;
  nav_loiter = false;

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
  // OBS!! Does not work
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
  // cout << "LONG DLON: " << (lon2 - lon1) << endl;
  // cout << "LONG DLAT: " << (lat2 - lat1) << endl;
  if( (lon2 - lon1) < 0){
    is_east = false;
    // cout << "vector is west" << endl;
  }
  else{
    is_east = true;
    // cout << "vector is pointing east" << endl;
  }
  if( (lat2 - lat1) < 0){
    is_north = false;
    // cout << "vector is pointing SOUTH" << endl;
  }
  else{
    is_north = true;
    // cout << "vector is pointing NORTH" << endl;
  }

  float lat1_rad = lat1*M_PI/180.0;
  float lat2_rad = lat2*M_PI/180.0;

  float a = sin(dlat_rad/2) * sin(dlat_rad/2) +
            cos(lat1_rad) * cos(lat2_rad) * sin(dlon_rad/2) * sin(dlon_rad/2);
  float c = 2 * atan2(sqrt(a),sqrt(1-a));
  float d = 6371e3 * c;  // distance in meters

  // cout << "Distance calculated between the coords: " << d << endl;

  return d; // return distance in meters
}
float DroneControl::get_distance_to_current_waypoint(){

  // return -1 if there is no more waypoints
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
  //cout << "dist lat: " << dist_lat/2 << endl;

  double dist_long = angles::to_degrees((next_long - current_long)/360);
  dist_long = dist_long * r;

  float distance_to_next =
      distance_between_two_coords(current_lat, current_long,
                                  next_lat, next_long);
  cout << "distance left: " << distance_to_next << endl;
  return distance_to_next;
}
float DroneControl::add_angles(float angle1, float angle2){

  float angle = angle1 + angle2;
  if(angle > 2.0*M_PI)
    angle -= M_PI;

  else if(angle < -0.0)
      angle += 2.0*M_PI;

  // cout << "added angle: " << angle << endl;
  return angle;
}
double DroneControl::get_bearing_to_current_waypoint_simple(){

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
  cout << "pos to wp current: " << current_lat << " " << current_lon << endl;
  float target_lat = waypoint_list[current_waypoint_index].x_lat;
  float target_lon = waypoint_list[current_waypoint_index].y_long;
  cout << "pos to wp target: " << target_lat << " " << target_lon << endl;

  current_lat = current_lat * M_PI/180.0;
  target_lat = target_lat * M_PI/180.0;
  float dlon = (target_lon - current_lon) * M_PI/180.0;
  float y = sin(dlon) * cos(target_lat);
  float x = cos(current_lat) * sin(target_lat) -
            sin(current_lat) * cos(target_lat) * cos(dlon);
  float angle = atan2(y, x);
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  // double heading = position_data[3];
  angle = fmod(((angle * 180/M_PI) + 360), 360);
  cout << "Bearing in complex method: " << angle << endl;
  cout << "Bearing in complex method: " << angles::to_degrees(angle_rad) << endl;
  return angle_rad; // return in radians
  //return angle; // return angle in degrees
}
double DroneControl::get_bearing_between_two_waypoints(float current_lat,
                                                       float current_lon,
                                                       float target_lat,
                                                       float target_lon){
  //cout << "current_waypoint_index" << current_waypoint_index << endl;

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  // cout << "wp to wp current: " << current_lat << " " << current_lon << endl;
  // cout << "wp to wp target: " << target_lat << " " << target_lon << endl;

  current_lat = current_lat * M_PI/180.0;
  target_lat = target_lat * M_PI/180.0;
  float dlon = (target_lon - current_lon) * M_PI/180.0;
  float y = sin(dlon) * cos(target_lat);
  float x = cos(current_lat) * sin(target_lat) -
            sin(current_lat) * cos(target_lat) * cos(dlon);
  float angle = atan2(y, x);
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  // double heading = position_data[3];
  angle = fmod(((angle * 180.0/M_PI) + 360), 360);
  // cout << "Bearing in wp to wp: " << angle << endl;
  // cout << "Bearing in wp to wp: " << angles::to_degrees(angle_rad) << endl;
  return angle_rad; // return in radians
  //return angle; // return angle in degrees
}
void DroneControl::calculate_velocity_angular(float bearing, float heading){

  double shortest_dist = angles::shortest_angular_distance(
    heading, bearing);
  double shortest_dist_abs = abs(shortest_dist);
  double err = bearing - heading;

  if(shortest_dist < 0)
    err = -err;

  double z_angular_velocity = pid_heading.calculate(0, shortest_dist);
  body_velocity.angular_z = z_angular_velocity;
}
void DroneControl::calculate_velocity_vertical(float target_height, float height){
  // target height is fixed to 5 meters atm.
  double vel_z = pid_height.calculate(target_height, height);
  body_velocity.linear_z = vel_z;
}
void DroneControl::set_velocity_body(){
  vector<float> position_data = tp.get_position_data();
  float height = position_data[2];

  move_msg.twist.linear.z = body_velocity.linear_z;

  if(height > 2){
    // move_msg.twist.angular.z = z_angular_velocity;
    move_msg.twist.angular.z = body_velocity.angular_z;
    // move_msg.twist.linear.x = 5.0 * x_vec;
    // move_msg.twist.linear.y = 5.0 * y_vec;
    move_msg.twist.linear.x = body_velocity.linear_x;
    move_msg.twist.linear.y = body_velocity.linear_y;
    // cout << "twist.linear.x: " << body_velocity.linear_x
    //  << " twist.linear.y: " << body_velocity.linear_y << endl;
  }

  tp.set_velocity(move_msg);
}
void DroneControl::calculate_velocity_body(float bearing,
                                           float bearing_pos_to_wp,
                                           float cross_track_err){

  calculate_velocity_angular(bearing, heading);
  calculate_velocity_vertical(target_height, height);
  
  double heading_x = cos(add_angles(double(-heading), M_PI/2.0));
  double heading_y = sin(add_angles(double(-heading), M_PI/2.0));
  double bearing_to_wp_x = cos(add_angles(double(-bearing_pos_to_wp), M_PI/2.0));
  double bearing_to_wp_y = sin(add_angles(double(-bearing_pos_to_wp), M_PI/2.0));
  double vel_x = heading_x * 5.0; // should be multiplied with some gain
  double vel_y = heading_y * 5.0; // related to max speed

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
  cout << "lateral_err" << lateral_err << endl;

  vel_x = vel_x + lateral_vel_x*lateral_err;
  vel_y = vel_y + lateral_vel_y*lateral_err;

  body_velocity.linear_x = vel_x;
  body_velocity.linear_y = vel_y;
}
#pragma region cross_track_err
float DroneControl::calculate_cross_track_error(float bearing_target,
                                                float distance_to_wp,
                                                float bearing_wp_to_wp){
float theta = bearing_wp_to_wp - bearing_target;
float theta_deg = angles::from_degrees(theta);
if(theta_deg > 270)
  theta += 2.0*M_PI;
else if(theta_deg > 90 and theta_deg < 270)
  cout << "OBS OBS!! THERE IS SOME ERROR HERE" << endl;
theta = abs(theta);
// float theta1 = (M_PI/2.0)-theta;
float cross_track_error = distance_to_wp * sin(theta);
// float cross_track_error1 = distance_to_wp * cos(theta1);
cout << "cross_track_error: " << cross_track_error << endl;

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
    // cout << "distance: " << distance << endl;
    if(distance < min_distance){
      chosen_index = i;
      min_distance = distance;
      // cout << "new min distance: " << min_distance << endl;
    }
  }
  return chosen_index;
}
void DroneControl::update_drone_position(){

  target_height = 5; // NEED TO MADE DIFFERENT!!
  float dist_to_next_wp = get_distance_to_current_waypoint();
  cout << "DISTANCE TO NEXT WP: " << dist_to_next_wp << endl;

  if(is_next_wp_loiter_wp(current_waypoint_index)){
    if(dist_to_next_wp < threshold_distance_to_loiter_waypoint && !nav_loiter){
      current_waypoint_index++;
    }
  }else if(waypoint_list[current_waypoint_index+1].command == MAV_CMD_NAV_LAND){
    target_height = 0;
  }
  
  if(dist_to_next_wp < threshold_distance_to_waypoint && !nav_loiter){
    current_waypoint_index++;
  }
  update_drone_data();
  // check if drone is passed the line segment, which it is tracking
  if(is_beyond_line_segment(previous_wp_lat, previous_wp_lon, next_wp_lat, next_wp_lon) && !nav_loiter){
    current_waypoint_index++;
  }

  float bearing = 0;
  float bearing_pos_to_wp = 0;
  float cross_track_error = 0;
  float wp0_lat = 0;
  float wp0_lon = 0;
  float wp1_lat = 0;
  float wp1_lon = 0;

  int MAV_CMD =  waypoint_list[current_waypoint_index].command;
  cout << "current command: " << MAV_CMD<< endl;
  cout << "current waypoint: " << current_waypoint_index+1 << endl;

  switch(MAV_CMD){
    case MAV_CMD_NAV_TAKEOFF:{
      float target_alt = waypoint_list[current_waypoint_index].z_alt; // target altitude
      body_velocity.linear_z =  pid_height.calculate(target_alt, height); //set height 0
      cout << "diff altitude: " << abs(target_alt - height) << endl;
      if(height > 0.5){
        wp0_lat = previous_wp_lat;
        wp0_lon = previous_wp_lon;
        wp1_lat = next_wp_lat;
        wp1_lon = next_wp_lon;
      }
      break;
    }
    case MAV_CMD_NAV_WAYPOINT:{
        wp0_lat = previous_wp_lat;
        wp0_lon = previous_wp_lon;
        wp1_lat = next_wp_lat;
        wp1_lon = next_wp_lon;
      break;
    }
    case MAV_CMD_NAV_LOITER_TIME:{
      // initialize first time it enters at every loiter waypoint
      if(!nav_loiter){
        // get loiter time from waypoint
        vector<float> position_data = tp.get_position_data();
        loiter_time = waypoint_list[current_waypoint_index+1].param1; // needs to be checked
        cout << "LOITER TIME: "<< loiter_time << endl;
        // set time ref
        start_time_loiter = clock();
        nav_loiter = true;
        // get loiter coordinates
        loiter_coordinates = get_loiter_target_coordinates(previous_wp_lat, previous_wp_lon, LOITER_RADIUS);

        // get first index/point for loiter
        // obtain point closest to uav
        current_loiter_index = get_closest_loiter_coordinate();

      }

      // cout << setprecision(9);
      // cout << "list of loiter coord:" << endl;
      // for(int i = 0; i < loiter_coordinates.size(); i++){
      //   cout << loiter_coordinates[i].lat << ", " << loiter_coordinates[i].lon << endl;
      // }
      float prev_loiter_lat = loiter_coordinates[current_loiter_index].lat;
      float prev_loiter_lon = loiter_coordinates[current_loiter_index].lon;

      int next_index = current_loiter_index + 1;
      if(current_loiter_index == loiter_coordinates.size() - 1){
        next_index = 0;
      }
      float next_loiter_lat = loiter_coordinates[next_index].lat;
      float next_loiter_lon = loiter_coordinates[next_index].lon;
      double dist_to_loiter_pt = distance_between_two_coords(cur_pos_lat, cur_pos_lon, next_loiter_lat, next_loiter_lon);
      cout << "DIST TO NEXT LOITER PT: " << dist_to_loiter_pt << endl;

      wp0_lat = prev_loiter_lat;
      wp0_lon = prev_loiter_lon;
      wp1_lat = next_loiter_lat;
      wp1_lon = next_loiter_lon;
      dist_to_next_wp = dist_to_loiter_pt;

      if(dist_to_loiter_pt < threshold_distance_to_waypoint || 
         is_beyond_line_segment(prev_loiter_lat, prev_loiter_lon, next_loiter_lat, next_loiter_lon)){
        if(current_loiter_index == loiter_coordinates.size()-1){
          current_loiter_index = 0;
        }else{
          current_loiter_index++;
        }
      }

      cout << "CURRENT LOITER INDEX: " << current_loiter_index << endl;
      double duration = ((clock() - start_time_loiter) / double(CLOCKS_PER_SEC)*10.0);
      cout << "duration: " << duration << " seconds" << endl;
      // cout << "duration compare: " << LOITER_TIME_SECONDS << endl;
      // loiter is done if duration is larger than the set loiter time
      if(duration > LOITER_TIME_SECONDS){
        cout << "STOP LOITER NOW!!" << endl;
        nav_loiter = false;
        // fly towards next waypoint by setting current wp as wp command.
        waypoint_list[current_waypoint_index].command = MAV_CMD_NAV_WAYPOINT;
      }
      break;
    }
    // case MAV_CMD_NAV_LOITER_UNLIM:{
    //   body_velocity.linear_z =  pid_height.calculate(0, height); //set height 0
    //   break;
    // }
    case MAV_CMD_NAV_LAND:{
      wp0_lat = previous_wp_lat;
      wp0_lon = previous_wp_lon;
      wp1_lat = next_wp_lat;
      wp1_lon = next_wp_lon;
      target_height = 0; 
      break;
    }
    default:{
      cout << "NO VALID COMMAND FROM MAVLINK - LAND AT LOCATION" << endl;
      target_height = 0;
      break;
      }
  }

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

  calculate_velocity_body(bearing, bearing_pos_to_wp,
                        cross_track_error);
  set_velocity_body();
  cout << endl << "------------------" << endl;
}

void DroneControl::update_drone_data(){
  // these must be initaliezed in the constructor
  vector<float> position_data = tp.get_position_data();
  cur_pos_lat = position_data[0];
  cur_pos_lon = position_data[1];
  previous_wp_lat = waypoint_list[current_waypoint_index].x_lat;;
  previous_wp_lon = waypoint_list[current_waypoint_index].y_long;
  next_wp_lat = waypoint_list[current_waypoint_index+1].x_lat;
  next_wp_lon = waypoint_list[current_waypoint_index+1].y_long;
  heading = angles::from_degrees(position_data[3]);
  height = position_data[2];
  //cout << "\n\nTIME: " << waypoint_list[current_waypoint_index+1].param1 << endl << endl;
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

    // cout << "pxr: "<< pxr << endl;
    // cout << "zxr: " << zxr << endl;

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