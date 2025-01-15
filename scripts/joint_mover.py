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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy
import moveit_commander
import numpy as np
import open3d
from time import sleep
from std_msgs.msg import String
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Vector3, Twist

def callback(position):
    pass


def callback_base(direction):
    print(direction.x)
    current_position = list(robot.get_current_state().joint_state.position)
    
    #0.3336 + 0.8
    desired_position = current_position[:]
    desired_position[0] = direction.x
    
    point.positions = desired_position
    point.time_from_start = rospy.Duration(1)
    
    traj.points.append(point)
    
    pub.publish(traj)
    
    


if __name__ == "__main__":
    
    rospy.init_node('ur5_rotation')
   
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)
    
    control_subscriber = rospy.Subscriber("/ur5_move_end_effector", Twist, callback)
    
    base_subscriber = rospy.Subscriber("/ur5_base_rotation", Vector3, callback_base)
    
    arm = moveit_commander.MoveGroupCommander('manipulator')
        # self.arm.set_planner_id("") # /home/acrv/HRIGroupAdmin/example_ros_ws/src/universal_robot/ur5_moveit_config/config/ompl_planning.yaml

    robot = moveit_commander.RobotCommander()

    # print(self.robot.get_group_names())
    # print('\n\n\n')
    # print(robot.get_current_state().joint_state.position)
   
    traj = JointTrajectory()
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                        'elbow_joint', 'wrist_1_joint', 
                        'wrist_2_joint', 'wrist_3_joint']
    current_position = [0.3613619804382324, -1.7017796675311487, -0.2897871176349085, -1.5448382536517542, -1.5679920355426233, 0.18697525560855865]
    current_position = [3.5952674807049334e-05, -1.57058030763735, 0.0001316070556640625, -1.5707486311541956, -0.00013143221010381012, -4.7985707418263246e-05]
    current_position = list(robot.get_current_state().joint_state.position)
    
    #0.3336 + 0.8
    desired_position = current_position[:]
    desired_position[0] = 0
    
    
    point = JointTrajectoryPoint()
    
    rospy.spin()
    