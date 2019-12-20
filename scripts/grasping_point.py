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

class MoveGroupPlanner():
    def __init__(self):
        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
        br = tf.TransformBroadcaster()
    

        #rospy.init_node('move_group_planner',
        #                anonymous=True)
 
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_left = moveit_commander.MoveGroupCommander("panda_left")
        self.group_right = moveit_commander.MoveGroupCommander("panda_right")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_left.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_left.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group_right.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group_right.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        rospy.sleep(1)
        stefan_dir = "/home/jiyeong/STEFAN/stl/"
        self.stefan = SceneObject()

        for key, value in self.stefan.list.items():
            self.scene.add_mesh(key, value, stefan_dir + key + ".stl")
            

        print ("============ Add Mesh:")
        ### Franka Collision
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        #self.set_collision_behavior.wait_for_service()

        self.active_controllers = []

        self.listener = tf.TransformListener()
        self.tr = TransformerROS()


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    def plan(self, goal, arm_name):
        if (arm_name == 'panda_left'):
            self.group_left.set_max_velocity_scaling_factor = 0.6
            self.group_left.set_max_acceleration_scaling_factor = 0.4
            self.group_left.set_start_state_to_current_state()
            trajectory = self.group_left.plan(goal)
        if (arm_name == 'panda_right'):
            self.group_right.set_max_velocity_scaling_factor = 0.6
            self.group_right.set_max_acceleration_scaling_factor = 0.4
            self.group_right.set_start_state_to_current_state()
            trajectory = self.group_right.plan(goal)
        return trajectory

    def plan_joint_target(self):
        joint_goal = self.group_left.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = -1.39117083 
        joint_goal[1] = -0.804575646 
        joint_goal[2] = 1.14795111
        joint_goal[3] = -1.76591437 
        joint_goal[4] = 0.745291994 
        joint_goal[5] = 1.51584423 
        joint_goal[6] = -0.477745851
        #self.group.set_pose_target(pose_goal) # it will be done by group.plan(pose_goal)        #self.group.set_pose_targets(...) # muiltiple EE
        #self.group.set_pose_targets(...) # muiltiple EE
        trajectory = self.group_right.plan(joint_goal)
        return trajectory
       
        
    def display_trajectory(self, plan):
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def check_grasp(self, part_name):
        file_name = self.stefan.stefan_dir + "../restefan/" + part_name + ".stl.yaml"
        with open(file_name, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        for pose in data_loaded['grasp_points']:
            grasp_point = geometry_msgs.msg.PoseStamped()
            grasp_point.header.frame_id = part_name + "_frame"
            grasp_point.header.stamp = rospy.Time.now()
            grasp_point.pose.position.x = pose['position'][0]
            grasp_point.pose.position.y = pose['position'][1]
            grasp_point.pose.position.z = pose['position'][2]
            grasp_point.pose.orientation.x = pose['orientation'][0]
            grasp_point.pose.orientation.y = pose['orientation'][1]
            grasp_point.pose.orientation.z = pose['orientation'][2]
            grasp_point.pose.orientation.w = pose['orientation'][3]
            rospy.sleep(1)
            grasp_point = listener.transformPose(self.planning_frame, grasp_point) #transfrom msg to "base" frame
            grasp_point.pose.position.z += 0.2
            self.group_left.set_pose_target(grasp_point)
            plan = self.group_left.plan()
            if plan.joint_trajectory.points:
                print(grasp_point)
                move_success = self.group_left.go()

            else:
                rospy.logerr("FAILED")

class TFLister:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')
    
    mdp = MoveGroupPlanner()
    listener = tf.TransformListener()
    listener.waitForTransform("short_part_frame", mdp.planning_frame, rospy.Time(0), rospy.Duration(4.0))
    
    mdp.check_grasp("long_part")

    # mdp.plan_joint_target()
    # mdp.group_right.go()
    # mdp.group_right.stop()
    rospy.spin()

    
    

