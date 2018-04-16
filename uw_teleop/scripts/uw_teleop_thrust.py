#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


import os.path as op


class UWTeleop(object):
    def __init__(self):
        rospy.init_node("keyboard")


        self.force_pub = rospy.Publisher('/NEXXUS_ROV/thrusters_input', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(20)


        self.robot_pose = rospy.Subscriber("/NEXXUS_ROV/pose",Pose,self.hold_thrust_callback)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_msg_callback)

        self.joy_data = Joy()
        self.vel_cmd = Float64MultiArray()
        self.past_pose = Pose()
        self.robot_pose = Pose()
        self.joy_called = 0
        self.set_pose = Pose()

    def hold_thrust_callback(self,data):
        self.robot_pose = data

        if self.joy_called == 1:
            if self.joy_data.buttons[1] == 1:
                self.set_pose = data

            if self.joy_data.buttons[1] == 0:
                x_gain = .1
                y_gain = .1
                z_gain = 0.1
                x_diff_g = 0.01
                y_diff_g = 0.01
                z_diff_g = 0.1
                x_port = (self.robot_pose.position.x - self.set_pose.position.x) * x_gain
                y_port = (self.robot_pose.position.y - self.set_pose.position.y) * y_gain
                z_port = (self.robot_pose.position.z - self.set_pose.position.z) * z_gain
                x_diff = (self.robot_pose.position.x - self.past_pose.position.x) * x_diff_g
                y_diff = (self.robot_pose.position.y - self.past_pose.position.y) * y_diff_g
                z_diff = (self.robot_pose.position.z - self.past_pose.position.z) * z_diff_g
                x_con = x_port + x_diff
                y_con = y_port + y_diff
                z_con = z_port + z_diff
                self.vel_cmd.data = [x_con,x_con,z_con,z_con,y_con]
                if self.robot_pose.position.z > 18.5:
                    self.vel_cmd.data = [x_con,x_con,-100,-100,y_con]
        self.past_pose = data

    def joy_msg_callback(self, data):
        '''
        joy_data.axes = [+left, +forward, +yaw, +up, +pitch, +roll]

        joy_data.buttons = [take_pictures, ...]
        '''

        self.joy_data = data
        self.joy_called = 1
        if self.joy_data.buttons[1]:
            x = self.joy_data.axes[1] * -10
            y = self.joy_data.axes[0] * 10
            z = self.joy_data.axes[3] * -10
            self.vel_cmd.data = [x,x,z,z,y]
            if self.robot_pose.position.z > 18.5:
                self.vel_cmd.data = [x,x,-100,-100,y]


    def start_teleop(self):
        while not rospy.is_shutdown():
            self.force_pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == "__main__":
    uw_teleop = UWTeleop()
    uw_teleop.start_teleop()
