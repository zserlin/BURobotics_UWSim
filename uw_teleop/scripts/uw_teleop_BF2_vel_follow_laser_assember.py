#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy, Image, LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf
from laser_geometry import LaserProjection
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import PyKDL
import os.path as op

from laser_assembler.srv import AssembleScans2

import copy

class UWTeleop(object):
    def __init__(self):
        rospy.init_node("uw_teleop")

        ##### try laser_assembler #####
        rospy.wait_for_service("assemble_scans2")
        self.assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        ###############################

        self.vel_pub1 = rospy.Publisher('/dataNavigator1', Odometry, queue_size=10)
        self.vel_pub2 = rospy.Publisher('/dataNavigator2', Odometry, queue_size=10)
        
        self.lidar1_pub = rospy.Publisher('/dataNavigatorlidar1', Odometry, queue_size=10)
        self.lidar2_pub = rospy.Publisher('/dataNavigatorlidar2', Odometry, queue_size=10)
        self.lidar3_pub = rospy.Publisher('/dataNavigatorlidar3', Odometry, queue_size=10)
        self.lidar4_pub = rospy.Publisher('/dataNavigatorlidar4', Odometry, queue_size=10)
        self.lidar5_pub = rospy.Publisher('/dataNavigatorlidar5', Odometry, queue_size=10)
        self.lidar6_pub = rospy.Publisher('/dataNavigatorlidar6', Odometry, queue_size=10)
        self.lidar7_pub = rospy.Publisher('/dataNavigatorlidar7', Odometry, queue_size=10)
        self.lidar8_pub = rospy.Publisher('/dataNavigatorlidar8', Odometry, queue_size=10)
        self.lidar9_pub = rospy.Publisher('/dataNavigatorlidar9', Odometry, queue_size=10)
        self.lidar10_pub = rospy.Publisher('/dataNavigatorlidar10', Odometry, queue_size=10)
        self.lidar11_pub = rospy.Publisher('/dataNavigatorlidar11', Odometry, queue_size=10)
        self.lidar12_pub = rospy.Publisher('/dataNavigatorlidar12', Odometry, queue_size=10)
        
        
        self.rate = rospy.Rate(20)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_msg_callback)

        self.bf1_odom_sub = rospy.Subscriber("/uwsim/BlueFox1Odom", Odometry, self.pose1_callback)
        self.bf2_odom_sub = rospy.Subscriber("/uwsim/BlueFox2Odom", Odometry, self.pose2_callback)

        self.bf1_lidar_sub = rospy.Subscriber("/lidar/multibeam", LaserScan, self.lidar_callback)
        #self.bf1_lidardup_sub = rospy.Subscriber("/lidar/multibeam", LaserScan, self.lidardup_callback)
        self.bf1_laser_sub = rospy.Subscriber("/BlueFox1/multibeam11", LaserScan, self.laser1_callback)
        self.bf2_laser_sub = rospy.Subscriber("/BlueFox2/multibeam", LaserScan, self.laser2_callback)
        
        
        # messages required by /ndt_mapping
        self.lidar_pc2_pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)
        self.imu_pub = rospy.Publisher("/imu_raw", Imu, queue_size=10)
        self.vehicle_odom_pub = rospy.Publisher('/odom_pose', Odometry, queue_size=10)
        
        self.joy_data = Joy()
        self.vel_cmd1 = Odometry()
        self.vel_cmd2 = Odometry()

        self.bf1_pose = None
        self.bf2_pose = None
        self.odom_hz = 10
        
        self.bf1_vel = None
        self.bf2_vel = None
        self.bf1_laser = LaserScan()
        self.bf2_laser = LaserScan()
        
        self.joy_button = None

        self.lidar_counter = 0 # this is a hack to make lidar spin

        self.laser_projection = LaserProjection()

        self.velodyne_mapped_laserscan = LaserScan()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def spin_lidar(self, T_bluefox, speed, counter, offset, lidar_odom_publisher):
        # calculate lidar pose
        T_lidar = np.dot(tf.transformations.rotation_matrix(np.pi/2, direction=(0,0,1)),
                         tf.transformations.rotation_matrix(np.pi/2, direction=(1,0,0)))
        #T_lidar = tf.transformations.euler_matrix(-np.pi/2, -np.pi/2, 0)
        # T_lidar[0,3] += 0.2
        # T_lidar[1,3] += 0.08
        T_lidar[2,3] += -0.15
        T_lidar_common = np.dot(T_bluefox, T_lidar)

        T_lidar = np.dot(T_lidar_common, tf.transformations.rotation_matrix(offset, direction=(0,1,0)))
        T_lidar = np.dot(T_lidar, tf.transformations.rotation_matrix(speed*counter, direction=(0,1,0)))
    
            
        quater_lidar = tf.transformations.quaternion_from_matrix(T_lidar)

        # make lidar follow
        lidar_odom = Odometry()

        ######## Position Control ############
        lidar_odom.pose.pose.position.x = T_lidar[0,3]
        lidar_odom.pose.pose.position.y = T_lidar[1,3]
        lidar_odom.pose.pose.position.z = T_lidar[2,3]
        
        lidar_odom.pose.pose.orientation.x = quater_lidar[0]
        lidar_odom.pose.pose.orientation.y = quater_lidar[1]
        lidar_odom.pose.pose.orientation.z = quater_lidar[2]
        lidar_odom.pose.pose.orientation.w = quater_lidar[3]
        ########################################

        lidar_odom_publisher.publish(lidar_odom)
        
    def pose1_callback(self, data):
        pose = data.pose.pose
        self.bf1_pose = pose
        self.bf1_vel = data.twist.twist

        T_bf1 = tf.transformations.quaternion_matrix([pose.orientation.x,
                                                      pose.orientation.y,
                                                      pose.orientation.z,
                                                      pose.orientation.w])
        
        T_bf1[:3,3] = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])

        # make lidar spin
        self.lidar_counter += 1
        if self.lidar_counter > 5000:
            self.lidar_counter = 0

        speed = 0.012
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=0.0,
                        lidar_odom_publisher=self.lidar1_pub)

        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=0.6,
                        lidar_odom_publisher=self.lidar2_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=1.2,
                        lidar_odom_publisher=self.lidar3_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=1.8,
                        lidar_odom_publisher=self.lidar4_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=2.4,
                        lidar_odom_publisher=self.lidar5_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=3.0,
                        lidar_odom_publisher=self.lidar6_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=3.6,
                        lidar_odom_publisher=self.lidar7_pub)

        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=4.2,
                        lidar_odom_publisher=self.lidar8_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=4.8,
                        lidar_odom_publisher=self.lidar9_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=5.4,
                        lidar_odom_publisher=self.lidar10_pub)
        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=6.0,
                        lidar_odom_publisher=self.lidar11_pub)

        self.spin_lidar(T_bf1,
                        speed=speed,
                        counter=self.lidar_counter,
                        offset=6.6,
                        lidar_odom_publisher=self.lidar12_pub)
   

        # publish a /odom_pose topic
        odom = Odometry()
        odom.pose = data.pose
        odom.twist = self.vel_cmd1.twist
        self.vehicle_odom_pub.publish(odom)
        
        # Publish a simulated Imu topic
        # imu = Imu()
        # imu.orientation = data.pose.pose.orientation

        
        
    def pose2_callback(self, data):
        pose = data.pose.pose
        self.bf2_pose = pose
        self.bf2_vel = data.twist.twist

    def lidar_callback(self, data):

        # T = self.tf_buffer.lookup_transform("lidar_multibeam", "imu", rospy.Time())
       
        # data.intensities = [1]*len(data.ranges)
        # pc = self.laser_projection.projectLaser(data,channel_options=self.laser_projection.ChannelOption.INTENSITY)
        # pc_transformed = do_transform_cloud(pc, T)
        # pc_transformed.header.frame_id = "velodyne"
        # pc_transformed.is_dense = True
        # self.lidar_pc2_pub.publish(pc_transformed)

        ##### try laser_assembler ####
        try:
            resp = self.assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            #print("Got cloud with %u points" %len(resp.cloud.data))
            self.lidar_pc2_pub.publish(resp.cloud)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
        ##############################
        
        pass

    def laser1_callback(self, data):
        self.bf1_laser = data
        
    def laser2_callback(self, data):
        self.bf2_laser = data

        
    def get_relative_pose(self, source, target):
        '''
        returns a vector pointing from source to target
        '''
        source_pos = np.array([source.position.x, source.position.y, source.position.z])
        target_pos = np.array([target.position.x, target.position.y, target.position.z])

        source_ori = source.orientation
        target_ori = target.orientation


        source_quater = (
            source_ori.x,
            source_ori.y,
            source_ori.z,
            source_ori.w
        )

        T_source = tf.transformations.quaternion_matrix(source_quater)
        T_source[:3,3] = source_pos

        target_quater = (
            target_ori.x,
            target_ori.y,
            target_ori.z,
            target_ori.w
        )

        
        T_target = tf.transformations.quaternion_matrix(target_quater)
        T_target[:3,3] = target_pos

 
        T_rel = np.dot(np.linalg.inv(T_source), T_target)
        
        return T_rel

    def get_planar_pose_follow_cmd(self, source, target, desired_dist=3.0):
        '''
        source vehicle follows target vehicle keeping a distance of 3.0 meters and maintaining the elevation
        '''

        
        T_rel = self.get_relative_pose(source, target)
        roll_rel, pitch_rel, yaw_rel = tf.transformations.euler_from_matrix(T_rel, axes='sxyz')
        rel_pos = T_rel[:3,3]
        current_dist = np.linalg.norm(rel_pos[:2])
        unit_rel_pos_xy = rel_pos[:2]/np.linalg.norm(rel_pos[:2])
        # q_rel = tf.transformations.quaternion_from_matrix(T_rel)

        vel_cmd = Odometry()

        p = 0.1
        vel_cmd.twist.twist.linear.z = p * rel_pos[2]

        if abs(np.linalg.norm(rel_pos[:2]) - desired_dist) < 0.01:
            vel_cmd.twist.twist.linear.x = 0
            vel_cmd.twist.twist.linear.y = 0
        elif np.linalg.norm(rel_pos[:2]) > desired_dist:
            vel_cmd.twist.twist.linear.x = p * unit_rel_pos_xy[0]
            vel_cmd.twist.twist.linear.y = p * unit_rel_pos_xy[1]
        else:
            vel_cmd.twist.twist.linear.x = -p * unit_rel_pos_xy[0]
            vel_cmd.twist.twist.linear.y = -p * unit_rel_pos_xy[1]

        vel_cmd.twist.twist.angular.x = 2.0 * p * roll_rel
        vel_cmd.twist.twist.angular.y = 2.0 * p * pitch_rel
        vel_cmd.twist.twist.angular.z = 2.0 * p * yaw_rel
            
        return vel_cmd

    def get_planar_vel_follow_cmd(self, target_vel):

        ''' source vehicle copies target vehicle's velocity'''
        vel_cmd = Odometry()
        
        vel_cmd.twist.twist = target_vel
        return vel_cmd

    def get_obs_avoidance_cmd(self, source_id, scale=0.01):
        if source_id == 1:
            laser = self.bf1_laser
            pose = self.bf1_pose
        if source_id == 2:
            laser = self.bf2_laser
            pose = self.bf2_pose

        angle = laser.angle_min
        angle_inc = laser.angle_increment    
        for i, r in enumerate(laser.ranges):
            r_vec = scale*np.array([np.cos(angle), np.sin(angle)])/r
            if i == 0:
                vel_vec = r_vec
            else:
                vel_vec = np.vstack([vel_vec, r_vec])

        vel_tot = np.sum(vel_vec, axis=0)

        vel_cmd = Odometry()

        vel_cmd.twist.twist.linear.x = -vel_tot[0]
        vel_cmd.twist.twist.linear.y = -vel_tot[1]

        return vel_cmd
            
    def joy_msg_callback(self, data):
        '''
        joy_data.axes = [+left, +forward, +yaw, +up, +pitch, +roll]

        joy_data.buttons = [take_pictures, ...]
        '''

        self.joy_data = data
        if self.joy_data.buttons[1] == 0:
            self.joy_button = 0

            self.vel_cmd1.twist.twist.linear.x = self.joy_data.axes[1] * .125
            self.vel_cmd1.twist.twist.linear.y = -self.joy_data.axes[0] * .125
            self.vel_cmd1.twist.twist.linear.z = -self.joy_data.axes[3] * .125
            self.vel_cmd1.twist.twist.angular.x = self.joy_data.axes[4] * .125
            self.vel_cmd1.twist.twist.angular.y = self.joy_data.axes[5] * .125
            self.vel_cmd1.twist.twist.angular.z = -self.joy_data.axes[2] * .125


        if self.joy_data.buttons[1] == 1:
            self.joy_button = 1

            self.vel_cmd2.twist.twist.linear.x = self.joy_data.axes[1] * .125
            self.vel_cmd2.twist.twist.linear.y = -self.joy_data.axes[0] * .125
            self.vel_cmd2.twist.twist.linear.z = -self.joy_data.axes[3] * .125
            self.vel_cmd2.twist.twist.angular.x = self.joy_data.axes[4] * .125
            self.vel_cmd2.twist.twist.angular.y = self.joy_data.axes[5] * .125
            self.vel_cmd2.twist.twist.angular.z = -self.joy_data.axes[2] * .125


    def start_teleop(self):
        while not rospy.is_shutdown():
            if self.joy_button == 0: # user is controlling BF1, BF2 is following
                follow_cmd = self.get_planar_pose_follow_cmd(source=self.bf2_pose, target=self.bf1_pose, desired_dist=3.0)
                obs_cmd = self.get_obs_avoidance_cmd(source_id=1, scale=0.01)

                self.vel_cmd2 = follow_cmd
                self.vel_cmd2.twist.twist.linear.x += obs_cmd.twist.twist.linear.x
                self.vel_cmd2.twist.twist.linear.y += obs_cmd.twist.twist.linear.y

                # self.vel_cmd2 = self.get_planar_vel_follow_cmd(target_vel=self.bf1_vel)
                
            # if self.joy_button == 1: # user is controlling BF1, BF2 is following
            #     rel_pos = self.get_relative_pose(source=self.bf1_pose, target=self.bf2_pose)
            #     #print(rel_pos)
            #     source_pos = np.array([self.bf1_pose.position.x, self.bf1_pose.position.y, self.bf1_pose.position.z])
            #     target_pos = np.array([self.bf2_pose.position.x, self.bf2_pose.position.y, self.bf2_pose.position.z])
            #     self.vel_cmd1 = self.get_planar_pose_follow_cmd(source_pos=source_pos, target_pos=target_pos,rel_pos=rel_pos,desired_dist=3.0)


            self.vel_pub1.publish(self.vel_cmd1)
            self.vel_pub2.publish(self.vel_cmd2)
            self.rate.sleep()


            
if __name__ == "__main__":
    uw_teleop = UWTeleop()
    uw_teleop.start_teleop()
