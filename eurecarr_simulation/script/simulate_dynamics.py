#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Accel
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

from tf.transformations import quaternion_from_euler
import tf_conversions
import tf2_ros

import numpy as np
import joystick

import sys
from time import gmtime, strftime

import dynamics



class SimulateStep(object):
    def __init__(self, stateDim, inputDim, dt):
        self.namespace  = ""
        # 0 1 2   3    4  5  6
        # x y yaw roll vx vy yawr

        '''
        Initial conditions
        '''
        self.x0          = 0.
        self.y0          = 0.
        self.yaw0        = 0.
        self.roll0       = 0.
        self.vx0         = 3.
        self.vy0         = 0.
        self.yawd0       = 0.

        '''
        Parameters
        '''
        self.dt          = dt
        self.fine_dt     = dt/10.
        self.length      = 0.12
        self.width       = 8.

        '''
        Variable initialization
        '''
        self.states_init = [self.x0, self.y0, self.yaw0, self.roll0, self.vx0, self.vy0, self.yawd0]
        self.stateDim    = stateDim
        self.inputDim    = inputDim
        self.dynamicsDim = 4
        self.states      = self.states_init
        self.states_der  = np.zeros(self.stateDim)
        self.state_hist  = np.zeros([1, (self.stateDim)])
        self.state_der_hist = np.zeros([1, (self.stateDim)])
        self.data        = np.zeros([1,2*self.dynamicsDim+self.inputDim])
        self.inputs      = np.zeros(self.inputDim)
        self.inputs_der  = np.zeros(self.inputDim)
        self.toggle_switch  = False

        '''
        Vehicle dynamics object
        '''
        self.dynamics    = dynamics.Dynamics(stateDim, inputDim, dt)
        self.dynamics.modelType = 1

        '''
        ROS publishers
        '''
        self.pose_pub   = rospy.Publisher('simulation/pose', PoseStamped, queue_size=1)
        self.bodyOdom_pub   = rospy.Publisher('simulation/bodyOdom', Odometry, queue_size=1)
        self.poseOdom_pub   = rospy.Publisher('simulation/mapOdom', Odometry, queue_size=1)
        self.input_pub  = rospy.Publisher('simulation/inputs', Joy, queue_size=1)
        self.accel_pub  = rospy.Publisher('simulation/acceleration', Accel, queue_size=1)
        self.br = tf2_ros.TransformBroadcaster()

        '''
        Variable definitions for autonomous driving
        '''
        self.auto_mode     = True
        self.control_sub   = rospy.Subscriber('control', AckermannDriveStamped, self.controlCallback)
        self.steering      = 0
        self.throttle      = 0
        self.steer_angle_to_norm = 1/0.25 #30/180*np.pi
        self.throttle_to_norm = 1

    def controlCallback(self, msg):
        '''
        Unit conventions
        - msg.drive.steering_angle : angle[rad]
        - msg.drive.acceleration   : acceleration[m/s^2]
        - self.inputs[0]           : normalized steer input(min:-1, max:1)
        - self.inputs[1]           : normalized throttle input(min:-1, max:1)
        '''
        self.steering = msg.drive.steering_angle * self.steer_angle_to_norm
        # if msg.drive.acceleration < 1.0 and msg.drive.acceleration > -1.0:
        self.throttle = msg.drive.acceleration * self.throttle_to_norm
        if self.auto_mode == True:
            self.inputs_d = np.array([self.steering, self.throttle])
            self.inputs += self.inputs_d * self.dt
            print("inputs: s %.3f t %.3f"%(self.inputs[0], self.inputs[1]))

    def simPoseSubCallback(self, msg):
        self.publishTF(self, msg.header.stamp, msg)

    def one_step_forward(self, inputs):
        '''
        Proceed one step
        '''
        if self.auto_mode == False:
            self.inputs     = inputs
        self.states_der = self.get_states_der(self.states, self.inputs)
        if self.toggle_switch:
            self.states_der = np.zeros_like(self.states)
            self.states     = self.states_init
        # self.states = self.states + self.states_der * self.dt # Euler method
        for _ in range(int(self.dt/self.fine_dt)): # Runge-Kutta method
            self.states = self.RK4(self.states, self.inputs, self.fine_dt)

        '''
        Stack history(optional)
        '''
        self.state_hist  = np.append(self.state_hist, [self.states], axis=0)
        self.state_der_hist  = np.append(self.state_der_hist, [self.states_der], axis=0)
        data_input = np.hstack([self.states[self.stateDim-self.dynamicsDim:], self.inputs])
        data_label = self.states_der[self.stateDim-self.dynamicsDim:]
        self.data = np.append(self.data,[np.hstack([data_input,data_label])],axis=0)

        '''
        Publish
        '''
        stamp = rospy.Time.now()

        self.publishPose(stamp, self.states)
        self.publishJoy(stamp, self.inputs, self.toggle_switch)
        self.publishBodyOdom(stamp, self.states)
        self.publishPoseOdom(stamp, self.states)
        self.publishAccel(stamp, self.states_der)

    def RK4(self, states, inputs, fine_Ts):
        # Runge-Kutta 4th Order Integration Method
        k1 = self.get_states_der(states, inputs)
        k2 = self.get_states_der(states+fine_Ts/2.*k1, inputs)
        k3 = self.get_states_der(states+fine_Ts/2.*k2, inputs)
        k4 = self.get_states_der(states+fine_Ts*k3, inputs)
        states_next = states + fine_Ts*(k1/6.+k2/3.+k3/3.+k4/6.)
        return states_next

    def get_states_der(self, states, inputs):
        return self.dynamics.forward(states, inputs)

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
        q = poseMsg.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.br.sendTransform(t)

    def publishPose(self, stamp, states):
        poseMsg = PoseStamped()
        poseMsg.header.frame_id  = "odom"
        poseMsg.header.stamp     = stamp
        poseMsg.pose.position.x  = states[0]
        poseMsg.pose.position.y  = states[1]
        poseMsg.pose.position.z  = 0.0
        '''
        State index and definition
        0 1 2   3    4  5  6
        x y yaw roll vx vy yawr
        '''
        roll  = states[3]
        pitch = 0.0
        yaw   = states[2]
        q     = quaternion_from_euler(roll, pitch, yaw)
        poseMsg.pose.orientation.x = q[0]
        poseMsg.pose.orientation.y = q[1]
        poseMsg.pose.orientation.z = q[2]
        poseMsg.pose.orientation.w = q[3]
        self.pose_pub.publish(poseMsg)
        self.publishTF(stamp, poseMsg)

    def publishJoy(self, stamp, inputs, toggle_switch):
        joyMsg = Joy()
        joyMsg.header.stamp   = stamp
        joyMsg.axes           = inputs
        joyMsg.buttons        = [toggle_switch]
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
        q     = quaternion_from_euler(roll, pitch, yaw)
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
        q     = quaternion_from_euler(roll, pitch, yaw)
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
        data = self.data
        name = "sim_data_"
        timelabel = strftime("%Y-%m-%d-%H-%M", gmtime())
        np.savetxt(name+timelabel+".csv", data, delimiter=",")



def main():
    rospy.init_node('simulate_dynamics')

    dt              = 0.05 # simulation timestep [sec]
    realtime_factor = 1.
    Hz              = int(1/dt)
    stateDim        = 7
    inputDim        = 2
    joy             = joystick.Joystick()
    sim             = SimulateStep(stateDim, inputDim, dt)
    sim.namespace   = rospy.get_namespace()
    rate            = rospy.Rate(Hz*realtime_factor)
    inputs = np.zeros(inputDim)

    joy_debug_flag = True
    run_debug_flag = True
    while not rospy.is_shutdown():
        joy.get_value()
        if(len(joy.axis)>=4):
            if run_debug_flag:
                print("Joystick activated.")
                run_debug_flag = False
                joy_debug_flag = True
            sim.inputs = np.array([joy.axis[3], -joy.axis[1]])
            sim.toggle_switch = joy.toggle
        else:
            if joy_debug_flag:
                print("Joystick not activated.")
                run_debug_flag = True
                joy_debug_flag = False
            sim.inputs = np.array([0.,0.])
        sim.one_step_forward(sim.inputs)

        rate.sleep()

    # Uncomment to save data on exit
    # rospy.on_shutdown(sim.save_state_hist)

if __name__ == "__main__":
    main()