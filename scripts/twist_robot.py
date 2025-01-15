#! /usr/bin/env python
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
import subprocess


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
from geometry_msgs.msg import Vector3, Twist
import sys
import math
import rospy
import tf
from tf import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_multiply, euler_from_quaternion, quaternion_from_euler

from tf.transformations import quaternion_matrix
import numpy as np


def callback(direction):
    global pub_twist
    # print(direction)
    scale_factor = 0.9
    twist = Twist()

    # Use TF to transform linear velocity from tool0_controller frame to base_link frame
    
    
    global listener
    
    ee_linear_vel = np.array([direction.linear.x, direction.linear.y, direction.linear.z])
    
    ee_angular_vel = np.array([direction.angular.y, direction.angular.z, direction.angular.x])
    
    # Wait for the transform to become available
    listener.waitForTransform("base", "tool0_controller", rospy.Time(0), rospy.Duration(5.0))
    (trans, rot) = listener.lookupTransform("base", "tool0_controller", rospy.Time(0))
    
    # listener.waitForTransform("base", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(5.0))
    # (trans, rot) = listener.lookupTransform("base", "camera_color_optical_frame", rospy.Time(0))
    
    rot_matrix = quaternion_matrix(rot)[0:3, 0:3]
    
    

    # Transform the linear velocity from tool0_controller to base_link frame
    base_linear_vel = rot_matrix.dot(ee_linear_vel)
    base_angular_vel = rot_matrix.dot(ee_angular_vel)
    

    # Assign transformed linear velocity to twist message
    twist.linear.x = base_linear_vel[0] *scale_factor
    twist.linear.y = base_linear_vel[1] *scale_factor
    twist.linear.z = base_linear_vel[2] *scale_factor
    
    twist.angular.x = base_angular_vel[0] *scale_factor
    twist.angular.y = base_angular_vel[1] *scale_factor
    twist.angular.z = base_angular_vel[2] *scale_factor
    
    # twist.linear.x = direction.linear.x * scale_factor
    # twist.linear.y = direction.linear.y * scale_factor
    # twist.linear.z = direction.linear.z * scale_factor
    
    # twist.angular.x = direction.angular.z * scale_factor
    # twist.angular.y = direction.angular.y * scale_factor
    # twist.angular.z = direction.angular.x * scale_factor
    
    pub_twist.publish(twist)


    

old_grip_pos = None
    
def callback_gripper(gripper_position):
    global old_grip_pos
    
    global pub
    
    print(gripper_position.x)
    
    if gripper_position != old_grip_pos:
        if gripper_position.x < 0:
            command.rPR = 230
            command.rGTO = 1
        elif gripper_position.x == 0:
            command.rGTO = 0
        else:
            command.rGTO = 1
            command.rPR = 0
    
    # command.rPR = gripper_position.x
    
    #if (command.rPR >= 0) and (command.rPR <= 230):
    pub.publish(command)
    
    old_grip_pos = gripper_position
    
    
def callback_pose(position):
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'stop', 'twist_controller'],
    )
    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'start', 'scaled_pos_joint_traj_controller'],
    )
    
    #code 
    
    waypoints = []
    
    wpose = move_group.get_current_pose().pose
    
    wpose.position.x = position.linear.x
    wpose.position.y = position.linear.y
    wpose.position.z = position.linear.z
    
    waypoints.append(copy.deepcopy(wpose))
    
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.1, 0.0)
        
    # raw_input('Check Rviz for plan, press enter to execute')
    move_group.execute(plan, wait = True)

    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'stop', 'scaled_pos_joint_traj_controller'],
    )
    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'start', 'twist_controller'],
    )
    



if __name__ == "__main__":
    
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    pub_twist = rospy.Publisher('/twist_controller/command', Twist, queue_size=1)
    control_subscriber = rospy.Subscriber("/ur5_move_end_effector", Twist, callback)
    
    gripper_subscriber = rospy.Subscriber("/ur5_gripper_control", Vector3, callback_gripper)
    
    pose_subscriber = rospy.Subscriber("/ur5_pose_set", Twist, callback_pose)
    
    
    global command

    command = outputMsg.Robotiq2FGripper_robot_output()
        
    command.rACT = 0
    
    pub.publish(command)

    rospy.sleep(0.1)
    
    
    command.rACT = 1 # activate gripper
    command.rGTO = 1 # Enable position  
    command.rSP  = 50 # maximum speed  
    command.rFR  = 150 # Moderate force
    
    pub.publish(command)

    rospy.sleep(0.1)
    
    
    command.rPR = 0
    
    pub.publish(command)

    rospy.sleep(0.1)
    
    
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
    
    listener = tf.TransformListener()
    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'stop', 'twist_controller'],
    )
    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'start', 'scaled_pos_joint_traj_controller'],
    )
    
    goal_pose = {
            'shoulder_pan_joint': 0.011,
            'shoulder_lift_joint': -1.636,
            'elbow_joint': 1.556,
            'wrist_1_joint': -1.543,
            'wrist_2_joint': -1.551,
            'wrist_3_joint': -0.138
        }
    
    move_group.set_joint_value_target(goal_pose)

    plan = move_group.plan()

    raw_input('Check Rviz for plan, press enter to execute')

    move_group.execute(plan, wait=True)
    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'stop', 'scaled_pos_joint_traj_controller'],
    )
    
    result = subprocess.call(
        ['rosrun', 'controller_manager', 'controller_manager', 'start', 'twist_controller'],
    )
    
    
    rospy.spin()