/**
 * @file MessageInterface.cpp
 * @author Seungwook Lee <seungwook1024@gmail.com>
 * @date February, 2020
 * @copyright 2020 KAIST
 * @brief Odometry node
 *
 * @details An interface node that transforms messages from arduino to appropriate ROS standard/custom messages
 * 
 * @modification 2020-10-26 Seperated runstop subscription and control publication.
 **/


#include "MessageInterface.h"

namespace eurecarr_core
{
MessageInterface::MessageInterface()
{
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    
    //nh.getParam("") // param for controller priority 
    /**
    * 'mppi_controller': 1
    'constantSpeedController': 2
    'waypointFollower': 3
    'joystick': 4
    'OCS': 5
    'RC': 7
     */

    nhp.param("pwm_steering_neutral_", pwm_steering_neutral_, 1500.0);
    nhp.param("steering_nonlinearity", steering_nonlinearity_, 0.0);
    nhp.param("steering_gain", steering_gain_, 1.0);
    nhp.param("throttle_gain", throttle_gain_, 0.92);
    nhp.param("throttle_max", throttle_max_, 0.6);
    nhp.param("throttle_min", throttle_min_, -0.5);
    nhp.param("invert_steering", invert_steering_, false);
    nhp.param("invert_throttle", invert_throttle_, false);
    nhp.param("steering_lpf_hz", steering_lpf_hz_, 100.0);
    nhp.param("throttle_lpf_hz", throttle_lpf_hz_, 10.0);
    if(steering_gain_ < 0 || throttle_gain_ < 0){
	    ROS_WARN("Do not set steering or throttle gain below 0. Use 'invert_*_' instead.");
    }
    ROS_INFO("steering_gain set to %f", steering_gain_);

    // initialize variables
    rate_ = 50; // loop rate in Hz
    steering_ = 0;
    throttle_ = 0;
    runstop_bool_.data = false;
    control_count_ = 0;
    reset_count_ = 0;
    throttle_max_ = std::max(throttle_max_, 0.0);
    throttle_min_ = std::min(throttle_min_, 0.0);
    steering_lpf_.setParams(1.0/rate_, steering_lpf_hz_, (double) 0); 
    throttle_lpf_.setParams(1.0/rate_, throttle_lpf_hz_, (double) 0);

    // subscribe from arduino
    wheel_speed_lf_sub_ = nh.subscribe("wheel_speed_lf", 1, &MessageInterface::wheelSpeedlfCallback, this);      // subscriber for wheel speeds <- Arduino Serial node
    wheel_speed_rf_sub_ = nh.subscribe("wheel_speed_rf", 1, &MessageInterface::wheelSpeedrfCallback, this);
    wheel_speed_lr_sub_ = nh.subscribe("wheel_speed_lr", 1, &MessageInterface::wheelSpeedlrCallback, this);
    wheel_speed_rr_sub_ = nh.subscribe("wheel_speed_rr", 1, &MessageInterface::wheelSpeedrrCallback, this);
    steer_debug_sub_ = nh.subscribe("steer_debug", 1, &MessageInterface::steerDebugCallback, this);
    throttle_debug_sub_ = nh.subscribe("throttle_debug", 1, &MessageInterface::throttleDebugCallback, this);
    // subscribe from autorally(to repeat to arduino)
    //automan_autorally_sub_ = nh.subscribe("automan", 1, &MessageInterface::automanCallback, this);
    runstop_autorally_sub_ = nh.subscribe("runstop", 10, &MessageInterface::runstopCallback, this);     
    chassis_command_joy_sub_ = nh.subscribe("joystick/chassisCommand", 3, &MessageInterface::chassisCommandJoyCallback, this); 
    chassis_command_mppi_sub_ = nh.subscribe("mppi_controller/chassisCommand", 3, &MessageInterface::chassisCommandMPPICallback, this);
    chassis_command_wpt_sub_ = nh.subscribe("waypointFollower/chassisCommand", 3, &MessageInterface::chassisCommandWptCallback, this);
    chassis_command_csc_sub_ = nh.subscribe("constantSpeedController/speedCommand", 1, &MessageInterface::chassisCommandCSCCallback, this);
    // publishers to conform to [autorally_msgs::*]
    wheel_speeds_pub_ = nh.advertise<autorally_msgs::wheelSpeeds>("wheelSpeeds", 1);    // pub at 
    // publishers to arduino
    automan_arduino_pub_ = nh.advertise<std_msgs::Bool>("automan_std", 1);              // pub at 
    runstop_arduino_pub_ = nh.advertise<std_msgs::Bool>("runstop_std", 1);              // pub at 
    pwm_steer_cmd_pub_ = nh.advertise<std_msgs::Float64>("pwm_steer_cmd", 3);           // pub at 
    pwm_throttle_cmd_pub_ = nh.advertise<std_msgs::Float64>("pwm_throttle_cmd",3);      // pub at 
    // publisher to autorally
    chassis_state_pub_ = nh.advertise<autorally_msgs::chassisState>("chassisState", 3); // pub at 

    // dynamic reconfigure
    dynamic_reconfigure::Server<MessageInterfaceParamsConfig>::CallbackType cb;

    cb = boost::bind(&MessageInterface::ConfigCallback, this, _1, _2);

    dynServer_.setCallback(cb);

}

MessageInterface::~MessageInterface()
{
}

void MessageInterface::wheelSpeedlfCallback(const std_msgs::Float64Ptr& ws_lf_msg)
{
    double ws = ws_lf_msg->data;
    speed_lf_ = ws;
}
void MessageInterface::wheelSpeedrfCallback(const std_msgs::Float64Ptr& ws_rf_msg)
{
    double ws = ws_rf_msg->data;
    speed_rf_ = ws;
}
void MessageInterface::wheelSpeedlrCallback(const std_msgs::Float64Ptr& ws_lr_msg)
{
    double ws = ws_lr_msg->data;
    speed_lr_ = ws;
}
void MessageInterface::wheelSpeedrrCallback(const std_msgs::Float64Ptr& ws_rr_msg)
{
    double ws = ws_rr_msg->data;
    speed_rr_ = ws;

}
void MessageInterface::steerDebugCallback(const std_msgs::Float64 steer)
{
    steer_debug_ = steer;
}
void MessageInterface::throttleDebugCallback(const std_msgs::Float64 throttle)
{
    throttle_debug_ = throttle;
}

void MessageInterface::automanCallback(const std_msgs::Bool automan_msg)
{
    automan_bool_ = automan_msg;
    automan_arduino_pub_.publish(automan_bool_);
}
void MessageInterface::runstopCallback(autorally_msgs::runstop runstop_msg)
{
    double runstop = runstop_msg.motionEnabled;
    runstop_bool_.data = runstop;
}

void MessageInterface::publishControl()
{
    runstop_arduino_pub_.publish(runstop_bool_);
    wheel_speeds_.lfSpeed = speed_lf_;
    wheel_speeds_.rfSpeed = speed_rf_;
    wheel_speeds_.lbSpeed = speed_lr_;
    wheel_speeds_.rbSpeed = speed_rr_;
    wheel_speeds_pub_.publish(wheel_speeds_);

    // check if reconfigured
    if(hasNewDynamicParams_ == true){
        steering_lpf_.setParams(1.0/rate_, steering_lpf_hz_, steering_filtered_); 
        throttle_lpf_.setParams(1.0/rate_, throttle_lpf_hz_, throttle_filtered_);
        hasNewDynamicParams_ = false;
    }
    // adjust with gain
    steering_ = std::min(std::max(steering_, (double)-1.0), (double)1.0);
    throttle_ = std::min(std::max(throttle_, throttle_min_), throttle_max_);
    // Low pass filter
    steering_lpf_.getFilteredValue(steering_, steering_filtered_);
    throttle_lpf_.getFilteredValue(throttle_, throttle_filtered_);
    //ROS_INFO("steering = %f * %f", steering_, steering_gain_);
    //ROS_INFO("throttle = %f * %f", throttle_, throttle_gain_);
    if(runstop_bool_.data == false){
	    throttle_ = 0;
            throttle_filtered_ = 0;
    }
    // form
    //pwm_steer_cmd_.data = SERVO_ALPHA * steering_ + SERVO_BETA;
    pwm_steer_cmd_.data = SERVO_ALPHA * steering_filtered_ + pwm_steering_neutral_;
    pwm_throttle_cmd_.data = SERVO_ALPHA * throttle_filtered_ + SERVO_BETA;
    // publish to Arduino
    pwm_steer_cmd_pub_.publish(pwm_steer_cmd_);
    pwm_throttle_cmd_pub_.publish(pwm_throttle_cmd_);
    // publish to autorally
    chassis_state_.autonomousEnabled = automan_bool_.data;
    chassis_state_.runstopMotionEnabled = runstop_bool_.data;
    if(control_sender_ == "joystick")
    {
        chassis_state_.throttleCommander = "joystick";
        chassis_state_.steeringCommander = "joystick";
    }
    if(control_sender_ == "mppi_controller"){
        chassis_state_.throttleCommander = "mppi_controller";
        chassis_state_.steeringCommander = "mppi_controller";
        // control_sender_ = "a";
    }
    if(control_sender_ == "waypoint_follower"){
        //chassis_state_.throttleCommander = "waypoint_follower";
        chassis_state_.steeringCommander = "waypoint_follower";
    }
    if(control_sender_csc_ == "constantSpeedController"){
        chassis_state_.throttleCommander = "constantSpeedController";
    }
    chassis_state_.header.stamp = ros::Time::now();
    chassis_state_.steering = steering_filtered_;
    chassis_state_.throttle = throttle_filtered_;
    //chassis_state_.steering = steering_;
    //chassis_state_.throttle = throttle_;
    chassis_state_pub_.publish(chassis_state_);



    // Clear inputs to 0 if not received for long.
    reset_count_ = std::min(100, reset_count_+1);
    if(reset_count_ > 20){
        steering_ = 0;
        throttle_ = 0;
        steering_filtered_ = 0;
        throttle_filtered_ = 0;
        control_sender_ = "";
        control_sender_csc_ = "";
        chassis_state_.throttleCommander = "";
        chassis_state_.steeringCommander = "";
    }
}

void MessageInterface::chassisCommandMPPICallback(autorally_msgs::chassisCommand chassis_command_msg)
{
//    if(chassis_command_msg.sender == "mppi_controller")
//    {
        control_sender_ = "mppi_controller";
        // collect
        steering_command_mppi_ = chassis_command_msg.steering;
        throttle_command_mppi_ = chassis_command_msg.throttle;
        if(invert_steering_ == true){
		steering_command_mppi_ = -steering_command_mppi_;
	}
	if(invert_throttle_ == true){
		throttle_command_mppi_ = -throttle_command_mppi_;
	}
	steering_ = steering_nonlinearity_ * pow(steering_command_mppi_, 3.0) + (1-steering_nonlinearity_) * steering_command_mppi_;
	//steering_ = steering_command_mppi_;
        throttle_ = throttle_command_mppi_;
//    }
    steering_ = steering_gain_ * steering_;
    throttle_ = throttle_gain_ * throttle_;

    control_count_ = 0;
    reset_count_ = 0;
}

void MessageInterface::chassisCommandWptCallback(autorally_msgs::chassisCommand chassis_command_msg)
{
//    if(chassis_command_msg.sender == "waypoint_follower")
//    {
        control_sender_ = "waypoint_follower";
        // collect
        steering_command_wpt_ = chassis_command_msg.steering;
        throttle_command_wpt_ = chassis_command_msg.throttle;
	if(invert_steering_ == true){
		steering_command_wpt_ = -steering_command_wpt_;
	}
	steering_ = steering_nonlinearity_ * pow(steering_command_wpt_, 3.0) + (1-steering_nonlinearity_) * steering_command_wpt_;
        //steering_ = steering_command_wpt_;
	if(invert_throttle_ == true){
		throttle_command_wpt_ = -throttle_command_wpt_;
	}
        //throttle_ = throttle_command_wpt_;
//    }
    steering_ = steering_gain_ * steering_;
    throttle_ = throttle_gain_ * throttle_;

    control_count_ = 0;
    reset_count_ = 0;
}

void MessageInterface::chassisCommandCSCCallback(std_msgs::Float64 chassis_command_msg)
{
    control_sender_csc_ = "constantSpeedController";
    // collect
    throttle_command_csc_ = chassis_command_msg.data;
    if(invert_throttle_ == true){
	    throttle_command_csc_ = -throttle_command_csc_;
    }
    //steering_ = steering_command_wpt_;
    throttle_ = throttle_command_csc_;
    steering_ = steering_gain_ * steering_;
    throttle_ = throttle_gain_ * throttle_;
    control_count_ = 0;
    reset_count_ = 0;
}

void MessageInterface::chassisCommandJoyCallback(autorally_msgs::chassisCommand chassis_command_msg)
{
    double steering_joy, throttle_joy;
    // // collect
    // if(control_sender_ == "mppi_controller")
    // {
    //     steer = steering_command_mppi_;
    //     throttle = throttle_command_mppi_;
    // }
    // // else if(control_sender_ == "waypoint_follower")
    // else{
    //     steering_ = chassis_command_msg.steering;
    //     throttle_ = chassis_command_msg.throttle;
    // }

    if(control_count_ > 10)
    {
        control_sender_ = "joystick";
        steering_joy = chassis_command_msg.steering;
        throttle_joy = chassis_command_msg.throttle;
        if(invert_steering_ == true){
            steering_joy = -steering_joy;
        }
        if(invert_throttle_ == true){
            throttle_joy = -throttle_joy;
        }
	steering_joy = steering_nonlinearity_ * pow(steering_joy, 3.0) + (1-steering_nonlinearity_) * steering_joy;
	steering_ = steering_gain_ * steering_joy;
        throttle_ = throttle_gain_ * throttle_joy;
    }
    control_count_ = std::min(50, control_count_ + 1);
    reset_count_ = 0;
}

  void MessageInterface::ConfigCallback(const MessageInterfaceParamsConfig &config, uint32_t level)
  {
    steering_nonlinearity_ = config.steering_nonlinearity;
    steering_lpf_hz_ = config.steering_lpf_hz;
    throttle_lpf_hz_ = config.throttle_lpf_hz;
    hasNewDynamicParams_ = true;
    std::cout << "MessageInterface Got a config!!" << "steering_nonlinearity: " << steering_nonlinearity_ << ", steering_lpf_hz: " << steering_lpf_hz_ << ", throttle_lpf_hz: " << throttle_lpf_hz_ << std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MessageInterface");
  eurecarr_core::MessageInterface MessageInterface;
  ros::Rate r(50);
  while(ros::ok())
  {
    MessageInterface.publishControl();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
