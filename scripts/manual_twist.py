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
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_multiply, euler_from_quaternion, quaternion_from_euler
from tf.transformations import quaternion_matrix
import time

if __name__ == "__main__":
    
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    pub_twist = rospy.Publisher('/twist_controller/command', Twist, queue_size=1)
    
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
    
    # Create a Twist message (linear specified in tool0_controller frame)
    twist_msg = Twist()
    
    # Desired linear velocity in tool0_controller frame
    # Example: move along tool0_controller's X-axis
    ee_linear_vel = np.array([0.0, -0.1, 0.0])  # Currently zero, can be changed as needed
    
    # Angular velocity is correct as is (assume already in base frame or acceptable)
    # so we won't transform angular
    twist_msg.angular.x = 0.0  
    twist_msg.angular.y = 0.0  
    twist_msg.angular.z = 0
    
    # Use TF to transform linear velocity from tool0_controller frame to base_link frame
    listener = tf.TransformListener()
    # Wait for the transform to become available
    listener.waitForTransform("base", "tool0_controller", rospy.Time(0), rospy.Duration(5.0))
    (trans, rot) = listener.lookupTransform("base", "tool0_controller", rospy.Time(0))

    rot_matrix = quaternion_matrix(rot)[0:3, 0:3]

    # Transform the linear velocity from tool0_controller to base_link frame
    base_linear_vel = rot_matrix.dot(ee_linear_vel)

    # Assign transformed linear velocity to twist message
    twist_msg.linear.x = base_linear_vel[0]
    twist_msg.linear.y = base_linear_vel[1]
    twist_msg.linear.z = base_linear_vel[2]

    # Publish for 2 seconds
    start_time = time.time()
    while time.time() - start_time < 4.0:
        pub_twist.publish(twist_msg)
        rospy.sleep(0.01)  # Short delay (100 Hz)

    # Stop the robot
    rospy.loginfo("Stopping robot...")
    twist_msg = Twist()  # zero velocities
    pub_twist.publish(twist_msg)

    rospy.spin()
