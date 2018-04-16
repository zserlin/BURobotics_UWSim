#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy, Image, JointState
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2

import os.path as op


class UWTeleop_joint(object):
    def __init__(self):
        rospy.init_node("uw_teleop_joint")

        self.vel_pub = rospy.Publisher('/uwsim/joint_state_command', JointState, queue_size=30)
        self.rate = rospy.Rate(30)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_msg_callback)
        self.joy_data = Joy()
        self.vel_cmd = JointState()
        self.vel_cmd.name = ['left1','left2','left3','left4','left5','left6','left_F_1','left_F_2','right1','right2','right3','right4','right5','right6','right_F_1','right_F_2']
        self.vel_cmd.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def joy_msg_callback(self, data):
        '''
        joy_data.axes = [+left, +forward, +yaw, +up, +pitch, +roll]

        joy_data.buttons = [take_pictures, ...]
        '''
        self.joy_data = data


        if self.joy_data.buttons[2] == 1:
            self.vel_cmd.velocity[8] = -self.joy_data.axes[0]
            self.vel_cmd.velocity[9] = -self.joy_data.axes[1]
            self.vel_cmd.velocity[10] = -self.joy_data.axes[2]
            self.vel_cmd.velocity[11] = -self.joy_data.axes[5]

        if self.joy_data.buttons[4] == 1:
            self.vel_cmd.velocity[12] = -self.joy_data.axes[0]
            self.vel_cmd.velocity[13] = -self.joy_data.axes[1]
            self.vel_cmd.velocity[14] = -self.joy_data.axes[2]
            self.vel_cmd.velocity[15] = -self.joy_data.axes[5]

        if self.joy_data.buttons[3] == 1:
            self.vel_cmd.velocity[0] = -self.joy_data.axes[0]
            self.vel_cmd.velocity[1] = -self.joy_data.axes[1]
            self.vel_cmd.velocity[2] = -self.joy_data.axes[2]
            self.vel_cmd.velocity[3] = -self.joy_data.axes[5]
        if self.joy_data.buttons[5] == 1:
            self.vel_cmd.velocity[4] = -self.joy_data.axes[0]
            self.vel_cmd.velocity[5] = -self.joy_data.axes[1]
            self.vel_cmd.velocity[6] = -self.joy_data.axes[2]
            self.vel_cmd.velocity[7] = -self.joy_data.axes[5]




    def start_teleop(self):
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == "__main__":

    uw_teleop_joint = UWTeleop_joint()
    uw_teleop_joint.start_teleop()
