/**
 * @file TrajectoryPrediction.cpp
 * @date 2020-04-03
 * @author sw
 * 
 * @brief Code for generating and publishing predicted trajectory of other cars
 *        based on subscribed sensor information
 *  
 * **/
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <random>
#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp> 

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <eurecarr_perception/TrajectoryPredictionParamsConfig.h>


namespace eurecarr_perception
{
  class TrajectoryPrediction
  {
  public:
    TrajectoryPrediction(ros::NodeHandle nh);
    ~TrajectoryPrediction();

    typedef struct 
    {
      int id;
      std::vector<float> x_pos;
      std::vector<float> y_pos;
      std::vector<float> z_pos;
      //1-2-3 Euler angles
      std::vector<float> roll;
      std::vector<float> pitch;
      std::vector<float> yaw;
      //Quaternions
      std::vector<float> q0;
      std::vector<float> q1;
      std::vector<float> q2;
      std::vector<float> q3;
      //X-Y-Z velocity.
      std::vector<float> x_vel;
      std::vector<float> y_vel;
      std::vector<float> z_vel;
      //Body frame velocity
      std::vector<float> u_x;
      std::vector<float> u_y;
      //Euler angle derivatives
      std::vector<float> yaw_mder;
      //Current servo commands
      std::vector<float> steering;
      std::vector<float> throttle;
    } FullState;

    typedef struct
    {
      int id;
      int dt;
      int numTimesteps;
      std::vector<float> x_pos;
      std::vector<float> y_pos;
      std::vector<float> yaw;
        /* predicted trajectory of others */
    } Trajectory;

  private:

    boost::mutex access_guard_;
    
    void EgoSubCallback(nav_msgs::OdometryConstPtr pose);
    void OpponentSubCallback(nav_msgs::OdometryConstPtr pose);
    void openReferenceWaypoint(ros::NodeHandle nh);
    void getOpponentState();
    void getClosestWaypoint(nav_msgs::Odometry current_pose, std::vector<float> wptOneDim);
    void reorderWaypoints();
    void constructPrediction();
    void interpolateByDt();
    void publishWaypoints();
    void publishTrajectory();
    void ConfigCallback(const TrajectoryPredictionParamsConfig &config, uint32_t level);
/**
 * TODO: do dynamic reconfigure
 * 
 *     void dynRcfgCallback(eurecarr_perception::TrajectoryPredictionParamsConfig &config, int lvl);

    bool hasNewDynRcfg();
    eurecarr_perception::TrajectoryPredictionParamsConfig getDynRcfgParams();

 * 
 * **/

    ros::NodeHandle nh_;
    ros::Subscriber egoSub_;
    ros::Subscriber oppoSub_;
    ros::Publisher oppoTrajPub_;
    ros::Publisher refWptPub_;
    eurecarr_perception::TrajectoryPredictionParamsConfig trajectoryParams_;
    dynamic_reconfigure::Server<TrajectoryPredictionParamsConfig> dynServer_;

    int numTimesteps_;
    int hz_;
    float dt_;
    float v_; // opponent's speed
    float speed_error_;
    float prediction_mean_x_;
    float prediction_mean_y_;
    float prediction_std_x_;
    float prediction_std_y_;
    float prediction_e_x_;
    float prediction_e_y_;

    nav_msgs::Odometry ego_pose_;
    nav_msgs::Odometry current_pose_; // current pose of the opponent in world coordinate
    nav_msgs::Odometry relative_pose_; // current relative pose of the opponent in world coordinate
    nav_msgs::Odometry last_pose_;
    std::vector<float> wptOneDim_;// data[2*i] = x, data[2*i+1] = y
    std::vector<float> wptOneDim_re_;
    std::vector<float> wptOneDim_trim_;
    std::vector<float> wptOneDim_inter_;
    std::vector<float> travel_btw_wpt_; // distance to the next waypoint

    int closest_index_;
    bool hasNewTrajectoryParams_ = false;

    std::string target_frame_;
    std::string wptFilename_;

    std::default_random_engine generator_;


  };
};