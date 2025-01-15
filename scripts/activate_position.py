#! /usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import open3d
from time import sleep
from std_msgs.msg import String


from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import Constraints, JointConstraint


'''

joint_state: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: "base_link"
  name: 
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
  position: [0.33631631731987, -1.7021873633014124, 1.6242966651916504, -1.5456164518939417, -1.5683396498309534, 0.1872149407863617]
  velocity: []
  effort: []
multi_dof_joint_state: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: "base_link"
  joint_names: []
  transforms: []
  twist: []
  wrench: []
attached_collision_objects: []
is_diff: False



'''


if __name__ == "__main__":
     # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('move_subscriber', anonymous=True)
    
    
    robot = moveit_commander.RobotCommander()
    
    # initialise moveit planning scene
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    table_size = [2, 2, 0.87]
    
    global move_group
    
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = box_pose.pose.position.z - .44 # shift by table size/2
    box_name = "table"
    scene.add_box(box_name, box_pose, size=table_size)
    
    camera_constraints = Constraints()
    camera_constraints.name = 'camera'
    
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = 'wrist_2_joint'
    joint_constraint.position = 0
    joint_constraint.tolerance_above = 2.55
    joint_constraint.tolerance_below = 2.55
    joint_constraint.weight = 1
    
    camera_constraints.joint_constraints.append(joint_constraint)
    
    
    move_group.set_path_constraints(camera_constraints)
    
    # goal_pose = {
    #         'shoulder_pan_joint': 0.3363,
    #         'shoulder_lift_joint': -1.702,
    #         'elbow_joint': 1.624,
    #         'wrist_1_joint': -1.545,
    #         'wrist_2_joint': -1.568,
    #         'wrist_3_joint': 0.187
    #     }
    
    # goal_pose = {
    #         'shoulder_pan_joint': 0.011,
    #         'shoulder_lift_joint': -1.636,
    #         'elbow_joint': 1.556,
    #         'wrist_1_joint': -1.543,
    #         'wrist_2_joint': -1.551,
    #         'wrist_3_joint': -0.138
    #     }
    
    goal_pose = {
            'shoulder_pan_joint': 0.568,
            'shoulder_lift_joint':  -1.272,
            'elbow_joint': -1.762,
            'wrist_1_joint':  -1.629,
            'wrist_2_joint': 1.652,
            'wrist_3_joint':2.151
        }
    
    move_group.set_joint_value_target(goal_pose)

    plan = move_group.plan()

    raw_input('Check Rviz for plan, press enter to execute')

    move_group.execute(plan)
    
    
    rospy.spin()
        