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

def callback(position):
    pass

def callback_gripper(position):
    
    print(position.x)
    
    command.rPR = position.x
    
    pub.publish(command)

    rospy.sleep(0.1)
    



if __name__ == "__main__":
    
    rospy.init_node('Robotiq2FGripperSimpleController')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    
    control_subscriber = rospy.Subscriber("/ur5_move_end_effector", Twist, callback)
    
    gripper_subscriber = rospy.Subscriber("/ur5_gripper_control", Vector3, callback_gripper)
    
    global command

    command = outputMsg.Robotiq2FGripper_robot_output();
    

        
    command.rACT = 0
    
    pub.publish(command)

    rospy.sleep(0.1)
    
    
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    
    pub.publish(command)

    rospy.sleep(0.1)
    
    
    command.rPR = 0
    
    pub.publish(command)

    rospy.sleep(0.1)
    
    
    rospy.spin()
    
    
    
    
    
    
