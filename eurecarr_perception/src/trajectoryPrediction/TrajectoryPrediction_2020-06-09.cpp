/**
 * @file TrajectoryPrediction.cpp
 * @date 2020-04-03
 * @author sw
 * 
 * @brief Code for generating and publishing predicted trajectory of other cars
 *        based on subscribed sensor information
 *  
 * **/

#include <eurecarr_perception/TrajectoryPrediction/TrajectoryPrediction.h>
#include <eurecarr_perception/param_getter.h>

using namespace eurecarr_perception;

namespace eurecarr_perception {

  TrajectoryPrediction::TrajectoryPrediction(ros::NodeHandle nh)
  {
    // TODO: enable dynamic reconfigure
    // nh_.param<int>("NumberOfTimesteps", numTimesteps_, 200);
    // nh_.param<int>("hz", hz_, 50);
    // nh_.param<float>("dt", dt_, 0.02);
    nh_ = nh;
    nh_.param("numTimesteps", numTimesteps_, (int)210);
    nh_.param("hz", hz_, (int) 50);
    nh_.param("dt", dt_, (float) 0.02);
    nh_.param("opponentSpeed", v_, (float)2.0);
    nh_.param("targetFrame", target_frame_, std::string("odom"));


    egoSub_ = nh_.subscribe("ego_pose", 1, &TrajectoryPrediction::EgoSubCallback, this); // pose_estimate of ego vehicle
    oppoSub_ = nh_.subscribe("oppo_pose", 1, &TrajectoryPrediction::OpponentSubCallback, this); // TODO: give namespace "opponent/one/"
    oppoTrajPub_ = nh_.advertise<nav_msgs::Path>("/opponents/predicted_trajectory", 1);
    refWptPub_ = nh_.advertise<nav_msgs::Path>("waypoint/reference", 1);
    openReferenceWaypoint(nh_);

    
  }

  TrajectoryPrediction::~TrajectoryPrediction()
  {
  }

  void TrajectoryPrediction::openReferenceWaypoint(ros::NodeHandle nh)
  {
    nh.param("wptFilename", wptFilename_, std::string("waypoints"));

    std::ifstream wptFile;
    std::string path_package = ros::package::getPath("autorally_control");
    std::string path_local = nh.param<std::string>("waypoint_file_path", "$(find autorally_control)/launch/");
    //std::string filename = path_package + path_local;
    std::string filename = path_local;
    wptFile.open(filename);
    std::string curLine;
    std::cout << filename << std::endl;

//    m_paramTimer = m_nh.createTimer(ros::Rate(1),
//                   &GpsWaypoint::paramCallback, this);
    int i = 0;
    wptOneDim_.clear();

    while(getline(wptFile, curLine))
    {
      double x, y;
      geometry_msgs::Point pt;
      std::vector<std::string> strs;
      boost::split(strs, curLine, boost::is_any_of(","));

      if (strs.size() == 2)
      pt.x = boost::lexical_cast<double>(strs[0]);

      pt.y = boost::lexical_cast<double>(strs[1]);
      ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
      wptOneDim_.push_back(pt.x);
      wptOneDim_.push_back(pt.y);
    }
    wptFile.close();

    dynamic_reconfigure::Server<TrajectoryPredictionParamsConfig>::CallbackType cb;

    cb = boost::bind(&TrajectoryPrediction::ConfigCallback, this, _1, _2);

    dynServer_.setCallback(cb);

    ros::Duration d = ros::Duration(1.0);

    d.sleep();

    publishWaypoints();

  }

  void TrajectoryPrediction::EgoSubCallback(nav_msgs::OdometryConstPtr pose)
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
    // ROS_INFO("Subscribing ego pose estimate");
  }

  void TrajectoryPrediction::OpponentSubCallback(nav_msgs::OdometryConstPtr pose)
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
    current_pose_.twist.twist.linear.x = pose->twist.twist.linear.x;
    current_pose_.twist.twist.linear.y = pose->twist.twist.linear.y;
    current_pose_.twist.twist.linear.z = pose->twist.twist.linear.z;
    // ROS_INFO("Opponent's pose recieved");
    getOpponentState();
    getClosestWaypoint(current_pose_, wptOneDim_);
    reorderWaypoints();
    constructPrediction();
    interpolateByDt();
    publishTrajectory();
    last_pose_ = current_pose_;
  }

  void TrajectoryPrediction::getOpponentState()
  {
    /* @brief suppose opponent's global position is given 
    // float roll, pitch, yaw; // relative rpy from ego to opponent
    // tf2::Quaternion q_ego, q_rel, q_new;
    // tf2::convert(ego_pose_->pose.pose.orientation, q_ego);
    // tf2::convert(relative_pose_->pose.pose.orientation, q_rel);
    // q_new = q_rel * q_ego;
    // q_new.normalize();
    // tf2::convert(q_new, current_pose_->pose.pose.orientation)

    // current_pose_->pose.pose.orientation.x = relative_pose_->pose.pose.orientation.x + ego_pose_->pose.pose.orientation.x;
    // current_pose_->pose.pose.orientation.y = relative_pose_->pose.pose.orientation.y + ego_pose_->pose.pose.orientation.y;
    // current_pose_->pose.pose.orientation.z = relative_pose_->pose.pose.orientation.z + ego_pose_->pose.pose.orientation.z;
    // current_pose_->pose.pose.orientation.w = relative_pose_->pose.pose.orientation.w + ego_pose_->pose.pose.orientation.w;
    */
    // nh_.param("opponentSpeed", v_, (float)2.0);
    
  }

  void TrajectoryPrediction::getClosestWaypoint(nav_msgs::Odometry current_pose, std::vector<float> wptOneDim)
  {
    // ROS_INFO("Calculating closest waypoint");
    double min_dist = 1000000000;
    int idx = 0;
    for(int i = 0; i < int(wptOneDim.size()); i+=2){
      double x = wptOneDim.at(i);
      double y = wptOneDim.at(i+1);
      double dist = pow(current_pose_.pose.pose.position.x - x,2) + pow(current_pose_.pose.pose.position.y - y,2);
      if(dist < min_dist) {
          min_dist = dist;
          idx = i;
      }
    }
    closest_index_ = idx/2;
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
    // ROS_INFO("Calculating closest waypoint end %d", closest_index_);

  }

  void TrajectoryPrediction::reorderWaypoints()
  {
    int index;
    int length = wptOneDim_.size();
    int margin = length - closest_index_*2;
    
    wptOneDim_re_.clear();
    for (int i=0; i<length; i+=2){
      if (i < margin){
        index = i + closest_index_*2;
      }
      else{
        index = i - margin;
      }
      wptOneDim_re_.push_back(wptOneDim_.at(index));
      wptOneDim_re_.push_back(wptOneDim_.at(index+1));

    }
  }
  void TrajectoryPrediction::constructPrediction()
  {
    // ROS_INFO("Constructing prediction");
    float vx = current_pose_.twist.twist.linear.x;
    float vy = current_pose_.twist.twist.linear.y;
    float vz = current_pose_.twist.twist.linear.z;
    float v = std::max(sqrt(vx*vx + vy*vy),(float)0.1); // opponent's speed in m/s
    float travel = 0.0; // accumulated distance between waypoints
    float travel_btw_wpt = 0.0;
    float horizon = v * numTimesteps_ * dt_;
    int i = 0;
    // float cur_x = wptOneDim_re_.at(0);
    // float cur_y = wptOneDim_re_.at(1);
    float cur_x = current_pose_.pose.pose.position.x;
    float cur_y = current_pose_.pose.pose.position.y;

    wptOneDim_trim_.clear();
    travel_btw_wpt_.clear();
    while (travel < horizon){
      i += 1;
      float next_x = wptOneDim_re_.at(2*i);
      float diff_x = next_x - cur_x;
      float next_y = wptOneDim_re_.at(2*i+1);
      float diff_y = next_y - cur_y;
      travel_btw_wpt = sqrt(diff_x*diff_x + diff_y*diff_y);
      travel += travel_btw_wpt;
      travel_btw_wpt_.push_back(travel_btw_wpt);
      wptOneDim_trim_.push_back(cur_x);
      wptOneDim_trim_.push_back(cur_y);
      cur_x = next_x;
      cur_y = next_y;
    }
    wptOneDim_trim_.push_back(cur_x);
    wptOneDim_trim_.push_back(cur_y);

    // resulting i > horizon, i-1 < horizon
    // ROS_INFO("Constructing prediction end");
    // ROS_INFO("travel=%f, horizon=%f",travel,horizon);

  }
  void TrajectoryPrediction::interpolateByDt()
  {
    // ROS_INFO("Interpolating by dt");
    int num_inter = 1; // number of interpolation
    int size = wptOneDim_trim_.size();
    float inter_x;
    float inter_y;
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    wptOneDim_inter_.clear();
    start_x = wptOneDim_trim_.at(0);
    start_y = wptOneDim_trim_.at(1);
    for (int i=2; i<size; i+=2){
      num_inter = std::max(std::min((int)( travel_btw_wpt_.at(i/2-1) / ((v_+0.00001) * (float)dt_) ), numTimesteps_),1); // num of interpolation btw wpts
      end_x = wptOneDim_trim_.at(i);
      end_y = wptOneDim_trim_.at(i+1);
      for (int j=0; j<num_inter; j++){
        inter_x = start_x + (float)(j)/(float)num_inter*(end_x - start_x);
        inter_y = start_y + (float)(j)/(float)num_inter*(end_y - start_y);
        wptOneDim_inter_.push_back(inter_x);
        wptOneDim_inter_.push_back(inter_y);
      }
      start_x = end_x;
      start_y = end_y;
      if(wptOneDim_inter_.size() >= numTimesteps_ * 2){
        break;
      }
    }
    while (wptOneDim_inter_.size() < numTimesteps_ * 2){
      wptOneDim_inter_.push_back(inter_x);
      wptOneDim_inter_.push_back(inter_y);
    }
    // ROS_INFO("Interpolating by dt end");

  }
  void TrajectoryPrediction::publishWaypoints()
  {
    // ROS_INFO("aa");
    nav_msgs::Path reference_waypoint; 

    geometry_msgs::PoseStamped wpt;
    wpt.header.stamp = ros::Time::now();
    wpt.header.frame_id = target_frame_.c_str();
    // wpt.header.frame_id = "odom";
    reference_waypoint.header.stamp = wpt.header.stamp;
    reference_waypoint.header.frame_id = wpt.header.frame_id;
    int waypoint_length = wptOneDim_.size();
    for (int i=0; i<waypoint_length; i+=2){
      wpt.pose.position.x = wptOneDim_.at(i);
      wpt.pose.position.y = wptOneDim_.at(i+1);
      if(i == 0){
        wpt.pose.orientation.x = current_pose_.pose.pose.orientation.x;
        wpt.pose.orientation.y = current_pose_.pose.pose.orientation.y;
        wpt.pose.orientation.z = current_pose_.pose.pose.orientation.z;
        wpt.pose.orientation.w = current_pose_.pose.pose.orientation.w;
      }
      reference_waypoint.poses.push_back(wpt);
    }

    refWptPub_.publish( reference_waypoint );
  }
  void TrajectoryPrediction::publishTrajectory()
  {
    /**pose.header.stamp = ...;
pose.header.frame_id = ...;
pose.pose.position.x = ...;
[...]

path.header.stamp = ...;
path.header.frame_id = ...;
path.poses.push_back(pose);
**/
    // ROS_INFO("Publish trajectory");
    nav_msgs::Path predicted_trajectory;

    int size = wptOneDim_inter_.size();
    geometry_msgs::PoseStamped wpt_pose;
    ros::Time time = current_pose_.header.stamp; // time when opponent's pose was recieved
    float cur_x = current_pose_.pose.pose.position.x;
    float cur_y = current_pose_.pose.pose.position.y;
    float x0 = wptOneDim_inter_.at(0);
    float y0 = wptOneDim_inter_.at(1);

    predicted_trajectory.header.stamp = time;
    predicted_trajectory.header.frame_id = target_frame_.c_str();
    for (int i=0; i<size; i+=2){
      wpt_pose.header.seq = i/2+1;
      wpt_pose.header.stamp = time;
      wpt_pose.header.frame_id = target_frame_.c_str();
      // pose.header.frame_id = "odom";
      wpt_pose.pose.position.x = wptOneDim_inter_.at(i);
      wpt_pose.pose.position.y = wptOneDim_inter_.at(i+1);
      if(i == 0){
        wpt_pose.pose.orientation.x = current_pose_.pose.pose.orientation.x;
        wpt_pose.pose.orientation.y = current_pose_.pose.pose.orientation.y;
        wpt_pose.pose.orientation.z = current_pose_.pose.pose.orientation.z;
        wpt_pose.pose.orientation.w = current_pose_.pose.pose.orientation.w;
      }

      // wpt_pose.pose.position.x = -4.0;
      // wpt_pose.pose.position.y = -4.5;
      predicted_trajectory.header.stamp = wpt_pose.header.stamp;
      predicted_trajectory.poses.push_back(wpt_pose);
    }
    // ROS_INFO("Publishing predicted trajectory");
    // ROS_INFO("Publishing reference waypoints size = %d", size/2);

    oppoTrajPub_.publish( predicted_trajectory );
  }
  void TrajectoryPrediction::ConfigCallback(const TrajectoryPredictionParamsConfig &config, uint32_t level)
  {
    // numTimesteps_ = config.numTimesteps;
    hz_ = config.hz;
    dt_ = config.dt;
    // target_frame_ = config.targetFrame;
    // wptFilename_ = config.wptFilename;
    // v_ = config.estimated_speed;
    hasNewTrajectoryParams_ = true;
    std::cout << "Got a config!!" << std::endl;
  }
  // void TrajectoryPrediction::dynRcfgCallback(eurecarr_perception::TrajectoryPredictionParamsConfig &config, int lvl)
  // {
  //   boost::mutex::scoped_lock lock(access_guard_);
  //   trajectoryParams_.numTimesteps = config.numTimesteps;
  //   trajectoryParams_.hz = config.hz;
  //   trajectoryParams_.dt = config.dt;
  //   trajectoryParams_.wptFilePath = config.wptFilePath;
  //   hasNewTrajectoryParams_ = true;
  // }

  // bool TrajectoryPrediction::hasNewDynRcfg()
  // {
  //   boost::mutex::scoped_lock lock(access_guard_);
  //   return hasNewTrajectoryParams_;
  // }

  // eurecarr_perception::TrajectoryPredictionParamsConfig TrajectoryPrediction::getDynRcfgParams()
  // {
  //   boost::mutex::scoped_lock lock(access_guard_);
  //   hasNewTrajectoryParams_ = false;
  //   return trajectoryParams_;
  // }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TrajectoryPrediction");
  ros::NodeHandle nh("~");
  TrajectoryPrediction prediction(nh);
  SystemParams params;
  /**
   * TODO: do dynamic reconfigure
   * 
   *   // loadParams(&params, nh);

  //   //   int numTimesteps_;
  //   // double hz_;
  //   // double dt_;
  // numTimestep_ = params.numTimestep;
  // hz_ = params.hz;
  // dt_ = params.dt;
  // dynamic_reconfigure::Server<TrajectoryPredictionParamsConfig> srv;
  // dynamic_reconfigure::Server<TrajectoryPredictionParamsConfig>::CallbackType callback_f;
  // callback_f = boost::bind(&TrajectoryPrediction::dynRcfgCallback, _1, _2);
  // srv.setCallback(callback_f);
  // ROS_INFO("Starting to spin...");
   * 
   *  
   * **/

  ros::spin();
}
