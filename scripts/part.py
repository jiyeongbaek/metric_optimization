import geometry_msgs.msg
from tf.listener import TransformerROS
import tf
import rospy
import moveit_msgs.msg
class SceneObject():
    def __init__(self):
        self.stefan_dir = "/home/jiyeong/STEFAN/stl/"
       
        self.long_part = "long_part"
        self.long_part_pose = geometry_msgs.msg.PoseStamped()
        self.long_part_pose.header.frame_id="base"
        self.long_part_pose.pose.orientation.w = 1.0
        self.long_part_pose.pose.position.x = 0.35
        self.long_part_pose.pose.position.y = 0.15
        self.long_part_pose.pose.position.z = 0.601

        self.bottom_part = "bottom"
        self.bottom_part_pose = geometry_msgs.msg.PoseStamped()
        self.bottom_part_pose.header.frame_id="base"
        self.bottom_part_pose.pose.orientation.w = 1.0
        self.bottom_part_pose.pose.position.x = 0.8
        self.bottom_part_pose.pose.position.y = -0.5
        self.bottom_part_pose.pose.position.z = 0.601

        self.short_part = "short_part"
        self.short_part_pose = geometry_msgs.msg.PoseStamped()
        self.short_part_pose.header.frame_id="base"
        self.short_part_pose.pose.orientation.w = 1.0
        self.short_part_pose.pose.position.x = 0.35
        self.short_part_pose.pose.position.y = 0.3
        self.short_part_pose.pose.position.z = 0.601

        self.middle_part = "middle_part"
        self.middle_part_pose = geometry_msgs.msg.PoseStamped()
        self.middle_part_pose.header.frame_id="base"
        self.middle_part_pose.pose.orientation.w = 1.0
        self.middle_part_pose.pose.position.x = 0.65
        self.middle_part_pose.pose.position.y = 0.4
        self.middle_part_pose.pose.position.z = 0.605

        self.side_chair = "side_chair"
        self.side_chair_pose = geometry_msgs.msg.PoseStamped()
        self.side_chair_pose.header.frame_id="base"
        self.side_chair_pose.pose.orientation.w = 1.0
        self.side_chair_pose.pose.position.x = 0.2
        self.side_chair_pose.pose.position.y = -0.4
        self.side_chair_pose.pose.position.z = 0.601

        self.side_chair_r = "side_chair_r"
        self.side_chair_r_pose = geometry_msgs.msg.PoseStamped()
        self.side_chair_r_pose.header.frame_id="base"
        self.side_chair_r_pose.pose.orientation.w = 1.0
        self.side_chair_r_pose.pose.position.z = 0.602

        self.list = {self.short_part : self.short_part_pose, 
                    self.long_part : self.long_part_pose, 
                    self.bottom_part : self.bottom_part_pose, 
                    self.middle_part : self.middle_part_pose, 
                    self.side_chair : self.side_chair_pose}