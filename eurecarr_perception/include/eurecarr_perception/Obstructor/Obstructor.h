/**
 * @file Obstructor.cpp
 * @date 2020-04-03
 * @author sw
 * 
 * @brief obstructor code on a circular track
 *  
 * **/
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
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

#define PI 3.141592;

namespace eurecarr_perception
{
  class Obstructor
  {
  public:
    Obstructor(ros::NodeHandle nh);
    ~Obstructor();

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

    float TRACK_CENTER_x = -8.0;
    float TRACK_CENTER_y = 0.0;
    int OBSTRUCT_INDEX_RANGE; // number of waypoints
    int OBSTRUCT_INDEX_RANGE_MIN = 6;
    int NUM_OF_TRACKS = 3;
    int DEFAULT_TRACK;
    int REORDER_INDEX_OFFSET;

    int selected_track_; // 0 - in, 1 - mid, 2 - out
    std::vector<float> track_in_; // in one dimension
    std::vector<float> track_mid_;
    std::vector<float> track_out_;
    std::vector<float> wptOneDim_;
    std::vector<float> wptOneDim_re_;
    int closest_index_;

    nav_msgs::Odometry ego_pose_; // ego pose
    nav_msgs::Odometry current_pose_; // current opponent pose
    nav_msgs::Odometry last_pose_; // last opponent pose
    float current_angle_;
    float current_radius_;
    float opponent_pose_x_;
    float opponent_pose_y_;
    float opponent_speed_;

    void EgoSubCallback(nav_msgs::OdometryConstPtr pose);
    void OpponentSubCallback(nav_msgs::OdometryConstPtr pose);

    void CartToPolar(float x, float y, float* const& angle, float* const& radius);

    void OpenReferenceWaypoint();
    void SelectTrack();

    void GetClosestWaypoint(nav_msgs::Odometry current_pose, std::vector<float> wptOneDim, float& distance, int& current_index);
    void GetClosestWaypointPolar(nav_msgs::Odometry current_pose, std::vector<float> wptOneDim);
    void ReorderWaypoints();

    void PublishSelectedTrack();

    float lpf(float x, float w, float dt);

  private:

    ros::NodeHandle nh_;
    ros::Subscriber oppoSub_;
    ros::Subscriber egoSub_;
    ros::Publisher refWptPub_;
    ros::Publisher refWptLocalPub_;

    std::string target_frame_;
    std::string wptFilename_;

    float lpf_x;
    float last_time_;
    bool isObstruct_;
    
  };
};