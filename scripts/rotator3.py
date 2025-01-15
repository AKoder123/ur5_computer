#! /usr/bin/env python
import copy
import rospy
import moveit_commander
import numpy as np
import open3d
from time import sleep
from std_msgs.msg import String
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import Constraints, JointConstraint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
import tf
from tf.transformations import euler_from_matrix, euler_matrix, quaternion_from_matrix, quaternion_multiply, euler_from_quaternion, quaternion_from_euler


if __name__ == "__main__":
    
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('get_pose')
    
    robot = moveit_commander.RobotCommander()
    
    # initialise moveit planning scene
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    table_size = [2, 2, 0.87]
    
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
    
    waypoints = []
    
    wpose = move_group.get_current_pose().pose
    
    waypoints.append(copy.deepcopy(wpose))
    
    current_orientation = wpose.orientation
    current_quat = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
    
    # Convert current quaternion to RPY
    delta_roll, delta_pitch, delta_yaw = 0.0, 0.0, 0.0
    
    delta_pitch -= 0.4
    
    delta_yaw, delta_roll, delta_pitch = delta_roll, delta_pitch, delta_yaw
    
    # n_delta_yaw, n_delta_roll, n_delta_pitch = 0.0, 0.0, 0.0

    # # Modify RPY in the end-effector frame
    
    # n_delta_pitch += 0.4
   
    
    # delta_pitch = n_delta_yaw
    # delta_yaw = n_delta_roll
    # delta_roll = n_delta_pitch
                                                                                
    delta_quat_local = quaternion_from_euler(delta_roll, delta_pitch, delta_yaw)
    
    new_quat = quaternion_multiply(current_quat, delta_quat_local)
    
    # new_quat = quaternion_from_euler(roll, pitch, yaw)
    
    wpose.orientation.x = new_quat[0]
    wpose.orientation.y = new_quat[1]
    wpose.orientation.z = new_quat[2]
    wpose.orientation.w = new_quat[3]
    
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        
    raw_input('Check Rviz for plan, press enter to execute')
    move_group.execute(plan, wait = False)
    

    print(wpose)
    
    rospy.spin()