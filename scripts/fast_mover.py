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
import geometry_msgs


if __name__ == "__main__":
    
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    
    
    
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
    
    waypoints = []
    
    move_group.clear_pose_targets()
    
    wpose = move_group.get_current_pose().pose
    
    #**
    
    # waypoints.append(copy.deepcopy(wpose))
    
    # wpose.position.x += 0.1
    
    # waypoints.append(copy.deepcopy(wpose))

    # (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.1, 0.0)
    
    # # raw_input('Check Rviz for plan, press enter to execute')
    # move_group.execute(plan, wait = False)
    
    #***
    
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position = wpose.position
    target_pose.orientation = wpose.orientation
   
    target_pose.position.x += 0.2
    move_group.set_pose_target(target_pose)
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)
    success = move_group.go(wait=True)
    
    move_group.stop()
    move_group.clear_pose_targets()
    
    
   # rospy.spin()