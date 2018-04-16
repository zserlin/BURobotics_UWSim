#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2

import os.path as op


class UWTeleop(object):
    def __init__(self):
        rospy.init_node("uw_teleop2")

        self.image_dir_path = op.dirname(op.realpath(__file__+"/../"))
        self.image_dir_path = self.image_dir_path + "/images/"
        self.vel_pub = rospy.Publisher('/dataNavigator2', Odometry, queue_size=10)
        self.rate = rospy.Rate(20)

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_msg_callback)
        self.image_sub = rospy.Subscriber("/BlueFox2/camera1", Image, self.image_callback)

        self.joy_data = Joy()
        self.vel_cmd = Odometry()

        self.cv_bridge = CvBridge()

        self.image_count = 0
    def image_callback(self,img):
        try:
            self.cv2_img = self.cv_bridge.imgmsg_to_cv2(img, 'bgr8')
        except CvBridgeError:
            print(CvBridgeError)


    def joy_msg_callback(self, data):
        '''
        joy_data.axes = [+left, +forward, +yaw, +up, +pitch, +roll]

        joy_data.buttons = [take_pictures, ...]
        '''

        self.joy_data = data
        if self.joy_data.buttons[2] == 0 and self.joy_data.buttons[3] == 0 and self.joy_data.buttons[4] == 0 and self.joy_data.buttons[5] == 0 :
            self.vel_cmd.twist.twist.linear.x = self.joy_data.axes[1]
            self.vel_cmd.twist.twist.linear.y = -self.joy_data.axes[0]
            self.vel_cmd.twist.twist.linear.z = -self.joy_data.axes[3]
            self.vel_cmd.twist.twist.angular.x = self.joy_data.axes[4]
            self.vel_cmd.twist.twist.angular.y = self.joy_data.axes[5]
            self.vel_cmd.twist.twist.angular.z = -self.joy_data.axes[2]

        if self.joy_data.buttons[0] == 1:
            cv2.imwrite(self.image_dir_path+'image_'+str(self.image_count)+'.png', self.cv2_img)
            self.image_count += 1

    def start_teleop(self):
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == "__main__":
    uw_teleop = UWTeleop()
    uw_teleop.start_teleop()
