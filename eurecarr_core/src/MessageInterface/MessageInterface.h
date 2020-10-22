/**
 * @file MessageInterface.h
 * @author Seungwook Lee <seungwook1024@gmail.com>
 * @date February, 2020
 * @copyright 2020 KAIST
 * @brief Odometry node
 *
 * @details An interface node that transforms messages from arduino to appropriate ROS standard/custom messages
 **/

#include <math.h>

#include "ros/ros.h"
#include <autorally_msgs/chassisState.h>
#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace eurecarr_core
{
    /**
     * @class MessageInterface
     * @brief An interface node that transforms messages from arduino to appropriate ROS std messages and autorally custom messages
     * 
     * @topics
     * @@pub:
     * @@sub:
     * 
     * */
class MessageInterface
{
public:
    MessageInterface();
    ~MessageInterface();

    private:
    const double PI = 3.14159265; ///< Value for pi
    const double MAX_SERVO_VAL = 0.65; ///< Maximum servo value vehicle will steer
    const double SERVO_ALPHA = 500; ///< Coefficient for calculating steering angle
    const double SERVO_BETA = 1500; ///< Coefficient for calculating steering angle

    ////////////////////////////////////////////
    ros::Subscriber wheel_speed_lf_sub_;        // subscriber for wheel speeds <- Arduino Serial node
    ros::Subscriber wheel_speed_rf_sub_;
    ros::Subscriber wheel_speed_lr_sub_;
    ros::Subscriber wheel_speed_rr_sub_;
    ros::Subscriber steer_debug_sub_;
    ros::Subscriber throttle_debug_sub_;
    ros::Subscriber automan_autorally_sub_;
    ros::Subscriber runstop_autorally_sub_;     // subscriber for runstop [autorally_msgs/runstop]
    ros::Subscriber chassis_command_joy_sub_;       // subscriber for collecting chassisCommand from the joystick [autorally_msgs/chassisCommand]
    ros::Subscriber chassis_command_mppi_sub_;
    ros::Subscriber chassis_command_wpt_sub_;
    ros::Subscriber chassis_command_csc_sub_;

    ros::Publisher chassis_state_pub_;          // publisher to conform to [autorally_msgs/chssisState] -> WheelOdometry node
    ros::Publisher wheel_speeds_pub_;           // publisher to conform to [autorally_msgs/wheelSpees] -> WheelOdometry node
    ros::Publisher runstop_arduino_pub_;        // publisher sent to arduino -> Arduino serial node
    ros::Publisher automan_arduino_pub_;
    ros::Publisher pwm_steer_cmd_pub_;
    ros::Publisher pwm_throttle_cmd_pub_;
    ////////////////////////////////////////////

    std_msgs::Bool runstop_bool_;
    std_msgs::Bool automan_bool_;
    std_msgs::Float64 pwm_steer_cmd_;           // retrieve pwm value from chassisCommand
    std_msgs::Float64 pwm_throttle_cmd_;
    std_msgs::Float64 steer_debug_;
    std_msgs::Float64 throttle_debug_;
    autorally_msgs::chassisCommand chassis_command_joy_;
    autorally_msgs::chassisCommand chassis_command_mppi_;
    autorally_msgs::chassisCommand chassis_command_wpt_;
    autorally_msgs::chassisCommand chassis_command_csc_;
    autorally_msgs::chassisState chassis_state_;
    autorally_msgs::wheelSpeeds wheel_speeds_;
    bool debug_;            ///< Publish debug values when true
    bool using_sim_;        ///< True if simulator is being used
    double time_delay_;     ///< Delay for the angular velocity to account for platform response time
    double max_servo_val_;  ///< Maximum servo value that vehicle can steer to

    double length_;  ///< Length of vehicle wheel base
    double width_;   ///< Length of vehicle axle

    double servo_val_;  ///< Servo value

    std::__cxx11::string control_sender_;
    std::__cxx11::string control_sender_csc_;
    double steering_;
    double throttle_;
    double steering_gain_; 
    double throttle_gain_;
    bool invert_steering_;
    bool invert_throttle_;
    double steering_command_mppi_;  ///< Steering angle of vehicle
    double throttle_command_mppi_;  ///< reserved commands for multiple controller commands
    double steering_command_wpt_;  ///< Steering angle of vehicle
    double throttle_command_wpt_;  ///< reserved commands for multiple controller commands
    double throttle_command_csc_;  ///< 

    double turn_radius_;     ///< Turn radius of the vehicle's immediate path

    double speed_lf_;   ///< Speed of the front left wheel in m/s
    double speed_rf_;   ///< Speed of the front right wheel in m/s
    double speed_lr_;   ///< Speed of the back left wheel in m/s
    double speed_rr_;   ///< Speed of the back right wheel in m/s
    double avg_speed_;  ///< Average speed of the front two wheels in m/s


    void wheelSpeedlfCallback(const std_msgs::Float64Ptr& ws_lf);
    void wheelSpeedrfCallback(const std_msgs::Float64Ptr& ws_rf);
    void wheelSpeedlrCallback(const std_msgs::Float64Ptr& ws_lr);
    void wheelSpeedrrCallback(const std_msgs::Float64Ptr& ws_rr);
    void steerDebugCallback(const std_msgs::Float64 steer);
    void throttleDebugCallback(const std_msgs::Float64 throttle);

    void automanCallback(std_msgs::Bool automan_autorally_sub_);
    void runstopCallback(autorally_msgs::runstop runstop_autorally_sub_);
    void chassisCommandJoyCallback(autorally_msgs::chassisCommand chassis_command_joy_);
    void chassisCommandMPPICallback(autorally_msgs::chassisCommand chassis_command_mppi_);
    void chassisCommandWptCallback(autorally_msgs::chassisCommand chassis_command_wpt_);
    void chassisCommandCSCCallback(std_msgs::Float64 chassis_command_wpt_);
};
}
