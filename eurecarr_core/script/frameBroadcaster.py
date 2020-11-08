#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
# from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from autorally_msgs.msg import chassisState, runstop
from autorally_msgs.msg import pathIntegralStats

import tf_conversions
import tf2_ros
from tf.transformations import quaternion_from_euler
prefix=""

class LPF(object):
    def __init__(self, dt, w, raw_0):
        self.dt = dt
        self.w = w
        self.raw = raw_0
        self.filtered = raw_0
        self.last_raw = 0
        self.last_filtered = 0

    def lpf(self, raw):
        self.raw = raw
        a = 2*self.w / (3*self.w + 2/self.dt)
        self.filtered = a * self.raw + (1 - a) * self.last_filtered
        self.last_filtered = self.filtered
        self.last_raw = self.raw
        return self.filtered


class FrameBroadcaster:
    def __init__(self):
        self.prefix = ""
        self.first_call = True
        dt = 1.0/120.0
        w = 10 * 3.141592
        self.origin_x = -8.5
        self.max_slip = 0
        self.max_speed = 0
        self.cur_speed = 0
        self.cur_speed_b = 0
        self.cur_slip = 0
        self.lap_time_avg = 0
        self.lap_time_std = 0
        self.max_speed = 0
        self.avg_speed = 0
        self.x_lpf = LPF(dt,w,0.0)
        self.y_lpf = LPF(dt,w,0.0)
        self.z_lpf = LPF(dt,w,0.0)
        self.xa = 0.0
        self.ya = 0.0
        self.va = 0.0
        self.xb = 0.0
        self.yb = 0.0
        self.vb = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.roll = 0.0
        self.roll_rate = 0.0
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.heading_multiplier = 0.0
        self.roll_lpf = LPF(dt,w,0.0)
        self.pitch_lpf = LPF(dt,w,0.0)
        self.yaw_lpf = LPF(dt,w,0.0)
        self.roll_rate_lpf = LPF(dt,w,0.0)
        self.pitch_rate_lpf = LPF(dt,w,0.0)
        self.yaw_rate_lpf = LPF(dt,w,0.0)
        self.seq = 1
        self.cur_pose_odom = Odometry()
        self.last_pose_time = 0.0
        self.last_pose_msg = PoseStamped()
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.last_yaw = 0.0
        self.last_yaw_temp = 0.0
        self.last_yaw_rate = 0.0
        self.last_v_x = 0.0
        self.last_v_y = 0.0
        self.last_v_z = 0.0
        self.vx_lpf = LPF(dt,w,0.0)
        self.vy_lpf = LPF(dt,w,0.0)
        self.vz_lpf = LPF(dt,w,0.0)
        self.poseDelay = Float64()
        # rospy.Subscriber("pose", PoseStamped, self.poseSubCallback)
        # rospy.Subscriber("ground_pose", Pose2D, self.groundPoseSubCallback)
        # rospy.Subscriber("mppi_controller/subscribedPose", Odometry, self.mppiposeCallback)
        # rospy.Subscriber("lap_stats", pathIntegralStats, self.lapStatCallback)
        #self.speed_pub = rospy.Publisher('info/linear_speed', Float64, queue_size = 1)
        #self.slip_pub = rospy.Publisher("info/slip_angle", Float64, queue_size=1)
        #self.distance_pub = rospy.Publisher("info/distance", Float64, queue_size=1)
        #self.ttc_pub = rospy.Publisher("info/ttc", Float64, queue_size=1)
        #self.angle_pub = rospy.Publisher("info/angle_difference", Float64, queue_size=1)
        #self.yaw_pub = rospy.Publisher("info/yaw", Float64, queue_size=1)
        #self.yaw_rate_pub = rospy.Publisher("info/yaw_rate", Float64, queue_size=1)
        #self.pose_odom_pub = rospy.Publisher("pose_estimate", Odometry, queue_size=1)
        #self.accel_pub_x = rospy.Publisher("acceleration/x", Float64, queue_size=1)
        #self.accel_pub_y = rospy.Publisher("acceleration/y", Float64, queue_size=1)
        #self.accel_pub_z = rospy.Publisher("acceleration/z", Float64, queue_size=1)
        #self.delays_pub = rospy.Publisher("info/poseDelays", Float64, queue_size=1)



        rospy.Subscriber("pose_estimate", Odometry, self.odomSubCallback)
        self.pose_odom = Odometry()
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()

    # def groundPoseSubCallback(self, pose_msg):
        # self.yaw = self.yaw_lpf.lpf(pose_msg.theta)

    def odomSubCallback(self, odom_msg):
        self.pose_odom = odom_msg
        self.tfPub()

    def tfPub(self):
        self.poseToTF(self.pose_odom, "odom", self.prefix + "base_link")
        # laser_position_x = -0.3
        # laserPos = Odometry()
        # laserPos.header.stamp = self.pose_odom.header.stamp
        # laserPos.pose.pose.position.x = laser_position_x
        # laserPos.pose.pose.orientation.w = 1.0
        # self.poseToTF(laserPos, "laser", self.prefix + "base_link")

    def poseSubCallback(self, pose_msg):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        z = pose_msg.pose.position.z
        # x = self.x_lpf.lpf(x)
        # y = self.y_lpf.lpf(y)
        # z = self.z_lpf.lpf(z)
        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w

        cur_pose_time = pose_msg.header.stamp.secs + pose_msg.header.stamp.nsecs * pow(10,-9)
        global dt
        if self.first_call:
            dt = 1.0/120.0
            self.first_call = False
        else:
            if cur_pose_time - self.last_pose_time > 0:
                dt = cur_pose_time - self.last_pose_time
    
        v_x = (self.last_pose_msg.pose.position.x - x) / dt
        if np.abs(v_x - self.last_v_x) > 0.5:
            v_x = self.last_v_x + 0.5 * np.sign(v_x - self.last_v_x)
        v_y = (self.last_pose_msg.pose.position.y - y) / dt
        if np.abs(v_y - self.last_v_y) > 0.5:
            v_y = self.last_v_y + 0.5 * np.sign(v_y - self.last_v_y)
        v_z = (self.last_pose_msg.pose.position.z - z) / dt
        if np.abs(v_z - self.last_v_z) > 0.5:
            v_z = self.last_v_z + 0.5 * np.sign(v_z - self.last_v_z)
        # v_x = self.vx_lpf.lpf(v_x)
        # v_y = self.vy_lpf.lpf(v_y)
        # v_z = self.vz_lpf.lpf(v_z)
        

        # Update euler angles. These use the 1-2-3 Euler angle convention.

        q0 = qw
        q1 = qx
        q2 = qy
        q3 = qz
        # self.roll = np.arctan2(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0)
        # self.pitch = -np.arcsin(2*q1*q3 - 2*q0*q2)
        self.roll = 0.0
        self.pitch = 0.0
        # self.yaw = 3.14/2.0
        q = quaternion_from_euler(0,0,self.yaw)
        
        self.yaw = np.arctan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2)
        # Don't allow heading to wrap around
        # autorally_plant.cpp line 198
        # if (last_heading_ > 3.0 && full_state_.yaw < -3.0){
        #     heading_multiplier_ += 1;
        # }
        # else if (last_heading_ < -3.0 && full_state_.yaw > 3.0){
        #     heading_multiplier_ -= 1;
        # }
        # last_heading_ = full_state_.yaw;
        # full_state_.yaw = full_state_.yaw + heading_multiplier_*2*3.14159265359;

        if self.last_yaw_temp > 3.1 and self.yaw < -3.1:
            self.heading_multiplier += 1
        elif self.last_yaw_temp < -3.1 and self.yaw > 3.1:
            self.heading_multiplier -= 1
        self.last_yaw_temp = self.yaw
        self.yaw = self.yaw + self.heading_multiplier*2*3.14159265359
        # if abs(self.yaw - self.last_yaw) > 1 * 3.14159265359 / 180.0:
        #     self.yaw = self.last_yaw + 1 * 3.14159265359 / 180.0*np.sign(self.yaw - self.last_yaw)
        #     print('fff')
        # else:
        #     print('lll')
        # print(self.heading_multiplier)


        # self.roll = self.roll_lpf.lpf(self.roll)
        # self.pitch = self.pitch_lpf.lpf(self.pitch)
        # self.yaw = self.yaw_lpf.lpf(self.yaw)
        
        self.roll_rate = (self.roll - self.last_roll) / dt
        self.pitch_rate = (self.pitch - self.last_pitch) / dt
        self.yaw_rate = (self.yaw - self.last_yaw) / dt
        if abs(self.yaw_rate - self.last_yaw_rate) > 1:
            self.yaw_rate = self.last_yaw_rate + 1*np.sign(self.yaw_rate - self.last_yaw_rate)
            # print('fff')
        # else:
            # print('lll')

        self.yaw_rate = self.yaw_rate_lpf.lpf(self.yaw_rate)
        self.last_yaw_rate = self.yaw_rate

        self.cur_pose_odom.header.frame_id = "odom"
        self.cur_pose_odom.child_frame_id = self.prefix + "base_link"
        self.cur_pose_odom.header.seq = self.seq
        self.cur_pose_odom.header.stamp.secs = pose_msg.header.stamp.secs
        self.cur_pose_odom.header.stamp.nsecs = pose_msg.header.stamp.nsecs
        self.cur_pose_odom.pose.pose.position.x = x
        self.cur_pose_odom.pose.pose.position.y = y
        self.cur_pose_odom.pose.pose.position.z = z
        self.cur_pose_odom.pose.pose.orientation.x = qx
        self.cur_pose_odom.pose.pose.orientation.y = qy
        self.cur_pose_odom.pose.pose.orientation.z = qz
        self.cur_pose_odom.pose.pose.orientation.w = qw
        self.cur_pose_odom.twist.twist.linear.x = v_x
        self.cur_pose_odom.twist.twist.linear.y = v_y
        self.cur_pose_odom.twist.twist.linear.z = v_z
        self.cur_pose_odom.twist.twist.angular.x = self.roll_rate
        self.cur_pose_odom.twist.twist.angular.y = self.pitch_rate
        self.cur_pose_odom.twist.twist.angular.z = self.yaw_rate

        self.poseToTF(self.cur_pose_odom)
        
        #Process the pose to get statistics
        total_v = (v_x**2 + v_y**2)**.5
        self.va = total_v
        self.cur_speed = total_v
        if (total_v > self.max_speed):
            self.max_speed = total_v

        self.ax = (v_x - self.last_v_x) / dt
        self.ay = (v_y - self.last_v_y) / dt
        self.az = (v_z - self.last_v_z) / dt

        self.last_pose_msg.pose.position.x = x
        self.last_pose_msg.pose.position.y = y
        self.last_pose_msg.pose.position.z = z
        self.last_pose_time = cur_pose_time
        self.last_roll = self.roll
        self.last_pitch = self.pitch 
        self.last_yaw = self.yaw
        self.last_v_x = v_x
        self.last_v_y = v_y
        self.last_v_z = v_z

    def poseToTF(self, msg, parent_frame, child_frame):
        self.t.header.stamp = rospy.Time.now()#msg.header.stamp
        self.t.header.frame_id = parent_frame
        self.t.child_frame_id = child_frame
        self.t.transform.translation.x = msg.pose.pose.position.x
        self.t.transform.translation.y = msg.pose.pose.position.y
        self.t.transform.translation.z = msg.pose.pose.position.z
        self.t.transform.rotation.x = msg.pose.pose.orientation.x
        self.t.transform.rotation.y = msg.pose.pose.orientation.y
        self.t.transform.rotation.z = msg.pose.pose.orientation.z
        self.t.transform.rotation.w = msg.pose.pose.orientation.w
        self.br.sendTransform(self.t)

def main():
    rospy.init_node("frame_broadcaster", anonymous=True)
    prefix = rospy.get_namespace()
    if (len(prefix) > 2):
        prefix = prefix[1:-1] + "/"
    print("prefix = ", prefix[0:-1])
    frameBroadcaster = FrameBroadcaster()
    frameBroadcaster.prefix = prefix
    #rate = rospy.Rate(50) #  Hz
    #while not rospy.is_shutdown():
    #    frameBroadcaster.tfPub()

    #    rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()    
