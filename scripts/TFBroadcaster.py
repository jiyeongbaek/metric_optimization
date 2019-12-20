#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

from part import SceneObject

import yaml
import tf2_ros
import tf2_msgs.msg
class TFBroadcaster:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        stefan = SceneObject()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "base"
            for key, value in stefan.list.items():
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = key + "_frame"
                t.transform.translation = value.pose.position
                t.transform.rotation = value.pose.orientation
                tfm = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('broadcast')
    tfb = TFBroadcaster()
    rospy.spin()