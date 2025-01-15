#! /usr/bin/env python
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
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
import time




if __name__ == "__main__":
    
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    pub_twist = rospy.Publisher('/twist_controller/command', Twist, queue_size=1)
    # control_subscriber = rospy.Subscriber("/ur5_move_end_effector", Twist, callback)
    
    # gripper_subscriber = rospy.Subscriber("/ur5_gripper_control", Vector3, callback_gripper)
    
    
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
    
    # Create a Twist message
    twist_msg = Twist()
    
    # Set linear velocities (m/s)
    twist_msg.linear.x = 0  # Move along X-axis
    twist_msg.linear.y = 0.0  # No movement along Y-axis
    twist_msg.linear.z = 0.0  # No movement along Z-axis
    
    # Set angular velocities (rad/s)
    twist_msg.angular.x = 0.0  # Yaw
    twist_msg.angular.y = 0.0  # Pitch
    twist_msg.angular.z = -0.2  # Roll +ve = anticlockwise

    # Record the start time
    start_time = time.time()
    
    while time.time() - start_time < 2.0:
        pub_twist.publish(twist_msg)
        rospy.sleep(0.01)  # Short delay to maintain loop stability (100 Hz)

    # Stop the robot after 1 second
    rospy.loginfo("Stopping robot...")
    twist_msg = Twist()  # Stop rotation
    pub_twist.publish(twist_msg)

    rospy.spin()