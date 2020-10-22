/**
 * @file Obstructor.cpp
 * @date 2020-04-03
 * @author sw
 * 
 * @brief Code for generating and publishing predicted trajectory of other cars
 *        based on subscribed sensor information
 *  
 * **/

#include <eurecarr_perception/Obstructor/Obstructor.h>
#include <eurecarr_perception/param_getter.h>

using namespace eurecarr_perception;

namespace eurecarr_perception {

  Obstructor::Obstructor(ros::NodeHandle nh)
  {
    // TODO: enable dynamic reconfigure
    // nh_.param<int>("NumberOfTimesteps", numTimesteps_, 200);
    // nh_.param<int>("hz", hz_, 50);
    // nh_.param<float>("dt", dt_, 0.02);
    nh_ = nh;
    nh_.param("targetFrame", target_frame_, std::string("odom"));
    nh_.param("reorder_offset", REORDER_INDEX_OFFSET, 0);
    nh_.param("default_track", DEFAULT_TRACK, 1);
    nh_.param("obstruct_range", OBSTRUCT_INDEX_RANGE, 10);
    nh_.param("isObstruct", isObstruct_, true);
    last_time_ = ros::Time::now().toSec();

    egoSub_ = nh_.subscribe("ego_pose", 1, &Obstructor::EgoSubCallback, this); // pose_estimate of ego vehicle
    oppoSub_ = nh_.subscribe("oppo_pose", 1, &Obstructor::OpponentSubCallback, this); // TODO: give namespace "opponent/one/"
    refWptPub_ = nh_.advertise<nav_msgs::Path>("waypoint/reference", 1);
    refWptLocalPub_ = nh_.advertise<nav_msgs::Path>("waypoint/reference/local", 1);
    
    OpenReferenceWaypoint();

    
  }

  Obstructor::~Obstructor()
  {
  }

    void Obstructor::EgoSubCallback(nav_msgs::OdometryConstPtr pose)
  {
    ego_pose_.header.frame_id = pose->header.frame_id;
    ego_pose_.header.stamp = pose->header.stamp;
    ego_pose_.header.seq = pose->header.seq;
    ego_pose_.pose.pose.position.x = pose->pose.pose.position.x;
    ego_pose_.pose.pose.position.y = pose->pose.pose.position.y;
    ego_pose_.pose.pose.position.z = pose->pose.pose.position.z;
    ego_pose_.pose.pose.orientation.x = pose->pose.pose.orientation.x;
    ego_pose_.pose.pose.orientation.y = pose->pose.pose.orientation.y;
    ego_pose_.pose.pose.orientation.z = pose->pose.pose.orientation.z;
    ego_pose_.pose.pose.orientation.w = pose->pose.pose.orientation.w;

    float q0 = ego_pose_.pose.pose.orientation.w;
    float q1 = ego_pose_.pose.pose.orientation.x;
    float q2 = ego_pose_.pose.pose.orientation.y;
    float q3 = ego_pose_.pose.pose.orientation.z;
    // ROS_INFO("Subscribing ego pose estimate");
  }

  void Obstructor::OpponentSubCallback(nav_msgs::OdometryConstPtr pose)
  {
    // *relative_pose_ = *pose;
    // ROS_INFO("Subscribing opponent pose estimate");

    current_pose_.header.frame_id = pose->header.frame_id;
    current_pose_.header.stamp = pose->header.stamp;
    current_pose_.header.seq = pose->header.seq;
    current_pose_.pose.pose.position.x = pose->pose.pose.position.x;
    current_pose_.pose.pose.position.y = pose->pose.pose.position.y;
    current_pose_.pose.pose.position.z = pose->pose.pose.position.z;
    current_pose_.pose.pose.orientation.x = pose->pose.pose.orientation.x;
    current_pose_.pose.pose.orientation.y = pose->pose.pose.orientation.y;
    current_pose_.pose.pose.orientation.z = pose->pose.pose.orientation.z;
    current_pose_.pose.pose.orientation.w = pose->pose.pose.orientation.w;
    // ROS_INFO("Opponent's pose recieved");
    // CartToPolar(current_pose_.pose.pose.position.x, current_pose_.pose.pose.position.y, &current_angle_, &current_radius_);
    SelectTrack();
    ReorderWaypoints();
    PublishSelectedTrack();
    last_pose_ = current_pose_;
  }

  void Obstructor::OpenReferenceWaypoint()
  {
    // nh_.param("wptFilename", wptFilename_, std::string("waypoints_in"));

    std::ifstream wptFile;
    std::string path_package = ros::package::getPath("autorally_control");
    std::string path_local = nh_.param<std::string>("/eurecarr_perception/import_file", "/launch/");
    std::string filepath = path_package + path_local;
    std::string filename = "waypoints";
    std::vector<float> wptOneDim;
    int size[NUM_OF_TRACKS];

    for (int i = 0; i < 3; i++){
      switch (i)
      {
      case 0:
        filename = filepath + nh_.param<std::string>("track_in", "waypoints_new_in");
        break;
      case 1:
        filename = filepath + nh_.param<std::string>("track_mid", "waypoints_new_mid");
        break;
      case 2:
        filename = filepath + nh_.param<std::string>("track_out", "waypoints_new_out");
        break;
      default:
        std::cout << "waypoints a" << std::endl;
        break;
      }
      wptFile.open(filename);
      std::string curLine;
      std::cout << filename << std::endl;

  //    m_paramTimer = m_nh.createTimer(ros::Rate(1),
  //                   &GpsWaypoint::paramCallback, this);
      wptOneDim.clear();

      while(getline(wptFile, curLine))
      {
        double x, y;
        geometry_msgs::Point pt;
        std::vector<std::string> strs;
        boost::split(strs, curLine, boost::is_any_of(","));

        if (strs.size() == 2)
        pt.x = boost::lexical_cast<double>(strs[0]);

        pt.y = boost::lexical_cast<double>(strs[1]);
        // ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
        wptOneDim.push_back(pt.x);
        wptOneDim.push_back(pt.y);
      }
      // ROS_INFO("Openning waypoints 1");

      wptFile.close();
      switch (i)
      {
      case 0:
      ROS_INFO("Openning waypoints track_in");
        track_in_ = wptOneDim;
        size[0] = track_in_.size();
        break;
      case 1:
      ROS_INFO("Openning waypoints track_mid");
        track_mid_ = wptOneDim;
        size[1] = track_mid_.size();
        break;
      case 2:
      ROS_INFO("Openning waypoints track_out");
        track_out_ = wptOneDim;
        size[2] = track_out_.size();
        break;
      default:
        std::cout << "waypoints b" << std::endl;
        break;
      }
      // publishWaypoints();
      // ROS_INFO("Openning waypoints 5");

    }
    ROS_INFO("Completed openning waypoints, size = %d, %d, %d", size[0], size[1], size[2]);

  }

  void Obstructor::SelectTrack()
  {
    // TODO: make it polar

    int track_num;
    int waypoint_length[3];
    std::vector<float> wpt_to_test;
    std::vector<float> ego_distances;
    std::vector<float> oppo_distances;
    std::vector<int> ego_indexes;
    std::vector<int> oppo_indexes; // current indexes
    float distance;
    int oppo_index;
    int ego_index;

    // ROS_INFO("Start selecting tracks 1");
    for (int i = 0; i < NUM_OF_TRACKS; i++){
      switch (i)
      {
      case 0:
        wpt_to_test = track_in_;
        waypoint_length[i] = track_in_.size();
        break;
      case 1:
        wpt_to_test = track_mid_;
        waypoint_length[i] = track_mid_.size();
        break;
      case 2:
        wpt_to_test = track_out_;
        waypoint_length[i] = track_out_.size();
        break;
      default:
        break;
      }
      // ROS_INFO("SelectTrack: examining track %d", i);
      GetClosestWaypoint(current_pose_, wpt_to_test, distance, oppo_index);
      oppo_distances.push_back(distance);
      oppo_indexes.push_back(oppo_index);
      GetClosestWaypoint(ego_pose_, wpt_to_test, distance, ego_index);
      ego_distances.push_back(distance);
      ego_indexes.push_back(ego_index);
    }
    // ROS_INFO("Start selecting tracks 2");

    //compare indexes on opponent's track
    track_num = std::distance(oppo_distances.begin(), std::min_element(oppo_distances.begin(), oppo_distances.end()));
    ego_index = ego_indexes.at(track_num);
    oppo_index = oppo_indexes.at(track_num);


    // obstruct only if the opponent is closely behind
    if ((oppo_index < ego_index - OBSTRUCT_INDEX_RANGE_MIN && oppo_index + OBSTRUCT_INDEX_RANGE > ego_index)
          || waypoint_length[track_num] - oppo_index + ego_index < OBSTRUCT_INDEX_RANGE){
      // selected_track_ = track_num;
      selected_track_ = track_num;
      ego_index = ego_indexes.at(selected_track_);
      // ROS_INFO("OBSTRUCTING, track = %d, o_idx = %d, e_idx = %d, dist = %f, %f, %f", selected_track_, oppo_index, ego_index,
      //      oppo_distances[0], oppo_distances[1], oppo_distances[2]);
    }
    // if (oppo_index < ego_index){
    //   selected_track_ = track_num;
    //   ego_index = ego_indexes.at(selected_track_);
    //   ROS_INFO("OBSTRUCTING, track = %d, o_idx = %d, e_idx = %d, dist = %f, %f, %f", selected_track_, oppo_index, ego_index,
    //        oppo_distances[0], oppo_distances[1], oppo_distances[2]);
    // }
    else{
      // if not, select ego closest track
      // only if the opponent is cleared
      if((oppo_index > ego_index - OBSTRUCT_INDEX_RANGE_MIN && oppo_index < ego_index + OBSTRUCT_INDEX_RANGE_MIN)){
        selected_track_ = std::distance(ego_distances.begin(), std::min_element(ego_distances.begin(), ego_distances.end()));
        ego_index = ego_indexes.at(selected_track_);
      }
      else{
        selected_track_ = DEFAULT_TRACK;
        ego_index = ego_indexes.at(selected_track_);
      }
    }
    if(!isObstruct_){
      selected_track_ = DEFAULT_TRACK;
      ego_index = ego_indexes.at(selected_track_);
    }
    // ROS_INFO("Start selecting tracks 3");

    // Apply LPF
    float time_now = ros::Time::now().toSec();
    float dt = time_now - last_time_;
    selected_track_ = lpf(selected_track_, 1, dt);
    last_time_ = time_now;


    closest_index_ = ego_index; // global variable
    switch (selected_track_)
    {
    case 0:
      wptOneDim_ = track_in_;
      break;
    case 1:
      wptOneDim_ = track_mid_;
      break;
    case 2:
      wptOneDim_ = track_out_;
      break;
    
    default:
      wptOneDim_ = track_mid_;
      break;
    }

    // wptOneDim_ = wpt_to_test;
    // ROS_INFO("Completed selecting tracks, oppo_index = %d, ego_index = %d", oppo_index, ego_index);

  }

  void Obstructor::GetClosestWaypoint(nav_msgs::Odometry current_pose, std::vector<float> wptOneDim, float& distance, int& current_index)
  {
    // ROS_INFO("Calculating closest waypoint");
    double min_dist = 1000000000;
    int idx = 0;
    int length = wptOneDim.size();
    // ROS_INFO("GetClosestWaypoint: waypoint length = %d", length);
    for(int i = 0; i < length; i+=2){
      double x = wptOneDim.at(i);
      double y = wptOneDim.at(i+1);
      double dist = pow(current_pose.pose.pose.position.x - x,2) + pow(current_pose.pose.pose.position.y - y,2);
      if(dist < min_dist) {
          min_dist = dist;
          idx = i;
      }
    }
    current_index = idx/2;
    distance = min_dist;
    //

    // std::vector<float> relative_dist_to_wpt;
    // int size = wptOneDim_.size();
    // for (int i=0; i<size; i+=2){
    //   float x = wptOneDim_.at(i) - current_pose_.pose.pose.position.x;
    //   float y = wptOneDim_.at(i) - current_pose_.pose.pose.position.y;
    //   relative_dist_to_wpt.push_back( pow(x, 2.0)+pow(y, 2.0) );
    // }
    // std::vector<float>::iterator result = std::min_element( relative_dist_to_wpt.begin(), relative_dist_to_wpt.end() );
    // closest_index_ = std::distance( relative_dist_to_wpt.begin(), result );
    // ROS_INFO("Calculating closest waypoint end %d", current_index);

  }

  void Obstructor::GetClosestWaypointPolar(nav_msgs::Odometry current_pose, std::vector<float> wptOneDim)
  {
    double current_x = current_pose.pose.pose.position.x;
    double current_y = current_pose.pose.pose.position.y;
    double current_angle = atan2(current_y - TRACK_CENTER_y, current_x - TRACK_CENTER_x);
    double min_angle = 3*PI;
    int idx = 0;
    for (int i =0; i < wptOneDim.size(); i+=2){

    }
  }

  void Obstructor::ReorderWaypoints()
  {
    int index;
    int length = wptOneDim_.size();
    int start_index;
    int index_offset = REORDER_INDEX_OFFSET;
    wptOneDim_re_.clear();

    // ROS_INFO("ReorderWaypoints: waypoint length = %d", length);
    if (closest_index_ < index_offset){
      start_index = length/2 + closest_index_ - index_offset; // if start index is below zero
    }
    else{
      start_index = closest_index_ - index_offset; // normal case
    }

    for (int i=0; i<length-2; i+=2){
      if (start_index*2 + i < length){
        index = i + start_index*2;
      }
      else{
        index = i + start_index*2 - length;
      }
      wptOneDim_re_.push_back(wptOneDim_.at(index));
      wptOneDim_re_.push_back(wptOneDim_.at(index+1));

    }
    // ROS_INFO("ReorderWaypoints end");

  }

  void Obstructor::PublishSelectedTrack()
  {
    // ROS_INFO("aa");
    nav_msgs::Path reference_waypoint; 

    geometry_msgs::PoseStamped wpt;
    wpt.header.stamp = ros::Time::now();
    wpt.header.frame_id = target_frame_.c_str();
    // wpt.header.frame_id = "odom";
    reference_waypoint.header.stamp = wpt.header.stamp;
    reference_waypoint.header.frame_id = wpt.header.frame_id;
    int waypoint_length = wptOneDim_re_.size();
    // give ego pose as first waypoint
    wpt.pose.position.x = ego_pose_.pose.pose.position.x;
    wpt.pose.position.y = ego_pose_.pose.pose.position.y;
    wpt.pose.orientation.w = ego_pose_.pose.pose.orientation.w;
    wpt.pose.orientation.x = ego_pose_.pose.pose.orientation.x;
    wpt.pose.orientation.y = ego_pose_.pose.pose.orientation.y;
    wpt.pose.orientation.z = ego_pose_.pose.pose.orientation.z;
    reference_waypoint.poses.push_back(wpt);


    // for (int i=0; i<waypoint_length; i+=2){
    float waypoint_length1 = std::min(20,waypoint_length);
    for (int i=2; i<waypoint_length1-2; i+=2){
      wpt.pose.position.x = wptOneDim_re_.at(i);
      wpt.pose.position.y = wptOneDim_re_.at(i+1);
      reference_waypoint.poses.push_back(wpt);
    }
    refWptPub_.publish( reference_waypoint );
    // ROS_INFO("Publishing reference waypoints");

    reference_waypoint.poses.clear();
    wpt.header.frame_id = "base_link";
    float heading;
    float q0 = ego_pose_.pose.pose.orientation.w;
    float q1 = ego_pose_.pose.pose.orientation.x;
    float q2 = ego_pose_.pose.pose.orientation.y;
    float q3 = ego_pose_.pose.pose.orientation.z;

    //Update euler angles. These use the 1-2-3 Euler angle convention.
    // full_state_.roll = atan2(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0);
    // full_state_.pitch = -asin(2*q1*q3 - 2*q0*q2);
    // full_state_.yaw = atan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);
    heading = atan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2);

    // ROS_INFO("heading = %f", heading);
    for (int i=0; i<(int)(waypoint_length*0.05)*2; i+=2){
      float wpt_x = wptOneDim_re_.at(i);
      float wpt_y = wptOneDim_re_.at(i+1);
      float ego_x = ego_pose_.pose.pose.position.x;
      float ego_y = ego_pose_.pose.pose.position.y;
      float costh = cosf(-heading);
      float sinth = sinf(-heading);
      float local_x = (wpt_x - ego_x)*costh - (wpt_y - ego_y)*sinth;
      float local_y = (wpt_x - ego_x)*sinth + (wpt_y - ego_y)*costh;
      // if(i==4) ROS_INFO("(dist, heading) = (%f, %f)", sqrt(local_x*local_x+local_y*local_y), heading);
      wpt.pose.position.x = local_x;
      wpt.pose.position.y = local_y;
      reference_waypoint.poses.push_back(wpt);
    }
    refWptLocalPub_.publish( reference_waypoint );
  }

  float Obstructor::lpf(float x, float w, float dt)
  {
    float a = 2*w/(3*w+2/dt);
    float lpf = a * x + (1-a) * lpf_x;
    lpf_x = x;
    return lpf;
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Obstructor");
  ros::NodeHandle nh("~");
  Obstructor prediction(nh);
  SystemParams params;

  ros::spin();
}