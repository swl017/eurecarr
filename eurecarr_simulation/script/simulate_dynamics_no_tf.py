#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Accel
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
# from tf.transformations import quaternion_from_euler
# import tf_conversions
# import tf2_ros

# code from: https://devnauts.tistory.com/61

import numpy as np
import pygame
from pygame.locals import *
import joystick
import sys
from time import gmtime, strftime

import dynamics

class SimulateStep(object):
    def __init__(self, stateDim, inputDim, dt):
        self.namespace  = ""
        # 0 1 2   3    4  5  6
        # x y yaw roll vx vy yawr

        self.dt          = dt
        self.img_name    = ''
        self.img         = None
        self.length      = 4.833
        self.width       = 1.921
        self.x0          = 0
        self.y0          = 0
        self.yaw0        = -1.
        self.roll0       = 0.0207
        self.vx0         = 1
        self.vy0         = 0.00086
        self.yawd0       = 0.0003
        self.states_init = [self.x0, self.y0, self.yaw0, self.roll0, self.vx0, self.vy0, self.yawd0]
        self.stateDim    = stateDim
        self.inputDim    = inputDim
        self.dynamicsDim = 4
        self.states      = self.states_init
        self.states_der  = np.zeros(self.stateDim)
        self.state_hist  = np.zeros([1, (self.stateDim)])
        self.state_der_hist  = np.zeros([1, (self.stateDim)])
        self.data        = np.zeros([1,2*self.dynamicsDim+self.inputDim])
        self.inputs      = np.zeros(self.inputDim)
        self.inputs_der  = np.zeros(self.inputDim)
        self.dynamics    = dynamics.Dynamics(stateDim, inputDim, dt)

        self.toggle      = False
        self.no_roll     = True
 
        self.pose_pub   = rospy.Publisher('simulation/pose', PoseStamped, queue_size=1)
        self.bodyOdom_pub   = rospy.Publisher('simulation/bodyOdom', Odometry, queue_size=1)
        self.poseOdom_pub   = rospy.Publisher('pose_estimate', Odometry, queue_size=1)
        self.input_pub  = rospy.Publisher('simulation/inputs', Joy, queue_size=1)
        self.accel_pub  = rospy.Publisher('acceleration/all', Accel, queue_size=1)
        # self.br = tf2_ros.TransformBroadcaster()

 
        # pygame.init() 
        # self.screen     = pygame.display.set_mode((480, 320), DOUBLEBUF)
        self.RED        = (255,0,0)
        self.BLACK      = (255,255,255)
        self.WHITE      = (0,0,0)


    def steerCallback(self, msg):
        self.steer_ = msg.data

    def one_step_forward(self, inputs):
        self.inputs     = inputs
        self.states_der = self.get_states_der(self.states, self.inputs)
        if self.toggle:
            self.states_der = np.zeros_like(self.states)
            self.states     = self.states_init
        self.states     = self.states + self.states_der * self.dt
        if self.no_roll == True:
            self.states[3] = 0
        self.state_hist  = np.append(self.state_hist, [self.states], axis=0)
        self.state_der_hist  = np.append(self.state_der_hist, [self.states_der], axis=0)
        data_input = np.hstack([self.states[self.stateDim-self.dynamicsDim:], self.inputs])
        data_label = self.states_der[self.stateDim-self.dynamicsDim:]
        self.data = np.append(self.data,[np.hstack([data_input,data_label])],axis=0)

        stamp = rospy.Time.now()

        self.publishPose(stamp, self.states)
        self.publishJoy(stamp, self.inputs, self.toggle)
        self.publishBodyOdom(stamp, self.states)
        self.publishPoseOdom(stamp, self.states)
        self.publishAccel(stamp, self.states_der)


    def get_states_der(self, states, inputs):
        return self.dynamics.forward(states, inputs)

    def render2D(self):
        x   = self.states[0]
        y   = self.states[1]
        yaw = self.states[2]

        center = (0.0, 0.0)
        front  = (self.length*1.3, 0)
        lf     = (self.length/2, self.width/2)
        rf     = (self.length/2, -self.width/2)
        lr     = (-self.length/2, self.width/2)
        rr     = (-self.length/2, -self.width/2)
        ce_rot = self.rotate(center, yaw)
        fr_rot = self.rotate(front, yaw)
        lf_rot = self.rotate(lf, yaw)
        rf_rot = self.rotate(rf, yaw)
        lr_rot = self.rotate(lr, yaw)
        rr_rot = self.rotate(rr, yaw)

        pygame.draw.line(self.screen, self.RED, ce_rot, fr_rot)
        pygame.draw.polygon(self.screen, self.RED, [lf_rot, rf_rot, lr_rot, rr_rot])

        # for i in range(0, len(self.state_hist)):
        #     pygame.draw.circle(self.screen, self.RED, [self.state_hist[i][0],self.state_hist[i][1]], float(self.width/3.0))
        self.screen.blit(self.img, (50, 100))
        degree = yaw * 180 / np.pi
        rotated = pygame.transform.rotate(self.img, degree)
        rect = rotated.get_rect()
        rect.center = (x, y)
        self.screen.blit(rotated, rect)



    def imageLoad(self, img_name):
        self.img_name = img_name
        self.img      = pygame.image.load(self.img_name)
        self.screen.fill(self.WHITE)
        self.screen = pygame.display.set_mode((480, 320), DOUBLEBUF)
        pygame.display.set_caption('Vehicle Dynamics Simulation')

    def rotate(self, pos, yaw):
        x_rot = np.cos(yaw)*pos[0] - np.sin(yaw)*pos[1]
        y_rot = np.sin(yaw)*pos[0] + np.cos(yaw)*pos[1]

        return x_rot, y_rot

    def publishTF(self, stamp, poseMsg):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = self.namespace[1:] + "base_link"
        t.transform.translation.x = poseMsg.pose.position.x
        t.transform.translation.y = poseMsg.pose.position.y
        t.transform.translation.z = 0.0
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, poseMsg.theta)
        q = poseMsg.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        # self.br.sendTransform(t)

    def publishPose(self, stamp, states):
        poseMsg = PoseStamped()
        poseMsg.header.frame_id  = "odom"
        poseMsg.header.stamp     = stamp
        poseMsg.pose.position.x  = states[0]
        poseMsg.pose.position.y  = states[1]
        poseMsg.pose.position.z  = 0.0
        # 0 1 2   3    4  5  6
        # x y yaw roll vx vy yawr
        roll  = states[3]
        pitch = 0.0
        yaw   = states[2]
        q     = self.quaternion_from_euler(roll, pitch, yaw)
        poseMsg.pose.orientation.x = q[0]
        poseMsg.pose.orientation.y = q[1]
        poseMsg.pose.orientation.z = q[2]
        poseMsg.pose.orientation.w = q[3]
        self.pose_pub.publish(poseMsg)
        self.publishTF(stamp, poseMsg)

    def publishJoy(self, stamp, inputs, toggle):
        joyMsg = Joy()
        joyMsg.header.stamp   = stamp
        joyMsg.axes           = inputs
        joyMsg.buttons        = [toggle]
        self.input_pub.publish(joyMsg)

    def publishBodyOdom(self, stamp, states):
        odomMsg = Odometry()
        odomMsg.header.frame_id       = "odom"
        odomMsg.header.stamp          = stamp
        odomMsg.pose.pose.position.x  = states[0]
        odomMsg.pose.pose.position.y  = states[1]
        odomMsg.pose.pose.position.z  = 0

        roll  = states[3]
        pitch = 0.0
        yaw   = states[2]
        q     = self.quaternion_from_euler(roll, pitch, yaw)
        odomMsg.pose.pose.orientation.x = q[0]
        odomMsg.pose.pose.orientation.y = q[1]
        odomMsg.pose.pose.orientation.z = q[2]
        odomMsg.pose.pose.orientation.w = q[3]

        odomMsg.twist.twist.linear.x   = states[4]
        odomMsg.twist.twist.linear.y   = states[5]
        odomMsg.twist.twist.linear.z   = 0.0
        roll_dot  = 0.0
        pitch_dot = 0.0
        yaw_dot   = states[6]
        odomMsg.twist.twist.angular.x = roll_dot
        odomMsg.twist.twist.angular.y = pitch_dot
        odomMsg.twist.twist.angular.z = self.states_der[2]#yaw_dot
        self.bodyOdom_pub.publish(odomMsg)

    def publishPoseOdom(self, stamp, states):
        odomMsg = Odometry()
        odomMsg.header.frame_id       = "odom"
        odomMsg.header.stamp          = stamp
        odomMsg.pose.pose.position.x  = states[0]
        odomMsg.pose.pose.position.y  = states[1]
        odomMsg.pose.pose.position.z  = 0

        roll  = states[3]
        pitch = 0.0
        yaw   = states[2]
        q     = self.quaternion_from_euler(roll, pitch, yaw)
        odomMsg.pose.pose.orientation.x = q[0]
        odomMsg.pose.pose.orientation.y = q[1]
        odomMsg.pose.pose.orientation.z = q[2]
        odomMsg.pose.pose.orientation.w = q[3]

        vx, vy = self.rotate([states[4], states[5]], yaw)
        odomMsg.twist.twist.linear.x   = vx
        odomMsg.twist.twist.linear.y   = vy
        odomMsg.twist.twist.linear.z   = 0.0
        roll_dot  = 0.0
        pitch_dot = 0.0
        yaw_dot   = states[6]
        odomMsg.twist.twist.angular.x = roll_dot
        odomMsg.twist.twist.angular.y = pitch_dot
        odomMsg.twist.twist.angular.z = self.states_der[2]#yaw_dot
        self.poseOdom_pub.publish(odomMsg)

    def publishAccel(self, stamp, states_der):
        accelMsg = Accel()
        accelMsg.linear.x  = states_der[4]
        accelMsg.linear.y  = states_der[5]
        accelMsg.linear.z  = 0.0
        accelMsg.angular.x = 0.0
        accelMsg.angular.y = 0.0
        accelMsg.angular.z = states_der[6]
        self.accel_pub.publish(accelMsg)

    def save_state_hist(self):
        # data = np.zeros(len(self.state_hist))
        # for i in range(0, len(self.state_hist)):
        #     data[i] = np.append(self.state_hist[i],self.state_der_hist[i])

        # data = np.append(self.state_hist, self.state_der_hist, axis=1)

        data = self.data
        name = "/home/sw/rosbag/csv/optitrack/2020-08-16/bicycle/SimpleBicycle_data_"
        timelabel = strftime("%Y-%m-%d-%H-%M", gmtime())
        np.savetxt(name+timelabel+".csv", data, delimiter=",")

    def quaternion_from_euler(self, roll, pitch, yaw):
        q = Quaternion()
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        q_list = [0,0,0,1]
        q_list[0] = q.x
        q_list[1] = q.y
        q_list[2] = q.z
        q_list[3] = q.w

        return q_list





def main():
    rospy.init_node('simulate_dynamics')

    dt           = 0.01
    Hz           = 1.0/dt
    stateDim     = 7
    inputDim     = 2
    joy          = joystick.Joystick()
    sim          = SimulateStep(stateDim, inputDim, dt)
    sim.namespace = rospy.get_namespace()
    # sim.imageLoad('/home/sw/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/kaist_costmap/waypoint_image3.png')
    clock        = pygame.time.Clock()
    r            = rospy.Rate(Hz)
    while not rospy.is_shutdown():
        joy.get_value()
        # joystick
        steering = joy.axis[3]
        throttle = -joy.axis[1]
        inputs = np.array([steering, throttle])
        sim.toggle = joy.toggle
        sim.one_step_forward(inputs)
        # clock.tick(Hz)
        # sim.render2D()

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        r.sleep()

    # rospy.on_shutdown(sim.save_state_hist)

if __name__ == "__main__":
    main()