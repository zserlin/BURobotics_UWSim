#!/usr/bin/env python

import rospy
import tf

import tf2_ros

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import numpy as np

class TfRemapper():

    def __init__(self):
    
        self.map_br = tf2_ros.TransformBroadcaster()
        self.baselink_br = tf2_ros.TransformBroadcaster()
        self.velodyne_br = tf2_ros.TransformBroadcaster()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)

        
    def tf_callback(self, data):
        
        map_tf = TransformStamped()
        map_tf.transform.rotation.w = 1.0
        map_tf.header.frame_id = "/world"
        map_tf.child_frame_id = "/map"
        map_tf.header.stamp = rospy.Time.now()
        self.map_br.sendTransform(map_tf)
        
        for tf in data.transforms:
            if tf.child_frame_id == "/BlueFox1":
                remapped_tf = self.copy_transform(tf, child_frame_id="/base_link")
                remapped_tf.header.frame_id = "/map"
                self.baselink_br.sendTransform(remapped_tf)


        velodyne_tf = TransformStamped()
        velodyne_tf.transform.rotation.x = np.sqrt(0.5)
        velodyne_tf.transform.rotation.y = 0
        velodyne_tf.transform.rotation.z = 0
        velodyne_tf.transform.rotation.w = np.sqrt(0.5)
        velodyne_tf.header.frame_id = "/base_link"
        velodyne_tf.child_frame_id = "/velodyne"
        velodyne_tf.header.stamp = rospy.Time.now()
        self.velodyne_br.sendTransform(velodyne_tf)

            
    def copy_transform(self, transform, child_frame_id=None):
        copied_tf = TransformStamped()
        copied_tf.header = transform.header
        if child_frame_id:
            copied_tf.child_frame_id = child_frame_id
        else:
            copied_tf.child_frame_id = transform.child_frame_id
        copied_tf.transform = transform.transform
        copied_tf.header.stamp = rospy.Time.now()
        
        return copied_tf
                
    def tf_remap(self):
         pass
        
if __name__ == "__main__":
    rospy.init_node("tf_remapper")
    remapper = TfRemapper()

    while not rospy.is_shutdown():
        remapper.tf_remap()
        #remapper.rate.sleep()
        rospy.spin()