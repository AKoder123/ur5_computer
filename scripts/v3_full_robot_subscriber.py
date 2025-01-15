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


old_direction = None
def callback(direction):
    global old_direction

    print(direction.angular)
    
    waypoints = []
    
    wpose = move_group.get_current_pose().pose

    if True:
        # Update rotation
        current_orientation = wpose.orientation
        current_quat = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        
        delta_roll, delta_pitch, delta_yaw = 0.0, 0.0, 0.0
        
        delta_roll += direction.angular.x * 2
        delta_pitch += direction.angular.y * 2
        delta_yaw += direction.angular.z * 2
        
        delta_yaw, delta_roll, delta_pitch = delta_roll, delta_pitch, delta_yaw
        
        delta_quat_local = quaternion_from_euler(delta_roll, delta_pitch, delta_yaw)

        new_quat = quaternion_multiply(current_quat, delta_quat_local)
        
        wpose.orientation.x = new_quat[0]
        wpose.orientation.y = new_quat[1]
        wpose.orientation.z = new_quat[2]
        wpose.orientation.w = new_quat[3]
        
        
        translation, rotation_quat = get_end_effector_transform()
        if translation is not None and rotation_quat is not None:
         
            new_dir = np.array([direction.linear.x, direction.linear.y, direction.linear.z])
            local_direction = transform_to_local_frame(new_dir, translation, rotation_quat)
        else:
            print("Failed to retrieve")
        # command.rPR = 230
        # command.rGTO = 1
        step = 1
        wpose.position.z -= local_direction[2] * step
        wpose.position.y += local_direction[1] * step
        wpose.position.x += local_direction[0] * step
        
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.1, 0.0)
        
        # raw_input('Check Rviz for plan, press enter to execute')
        move_group.execute(plan, wait = False)

    old_direction = direction
    

old_grip_pos = None
    
def callback_gripper(gripper_position):
    global old_grip_pos
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

def get_end_effector_transform():
    # rospy.init_node("frame_transformer", anonymous=True)
    listener = TransformListener()
    try:
        listener.waitForTransform('/base_link', '/tool0', rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = listener.lookupTransform('/base_link','/tool0', rospy.Time(0))
        return np.array(trans), np.array(rot)
    
    except tf.Exception as e:
        rospy.logerr(e)
        return
    
def transform_to_local_frame(global_vector, translation, rotation_quat):
    rotation = R.from_quat(rotation_quat)
    rotation_matrix=rotation.as_dcm()
    print(rotation_matrix)
    print(global_vector)
    local_vector = np.dot(rotation_matrix.T, global_vector)
    return local_vector


if __name__ == "__main__":
    
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    
    control_subscriber = rospy.Subscriber("/ur5_move_end_effector", Twist, callback)
    
    gripper_subscriber = rospy.Subscriber("/ur5_gripper_control", Vector3, callback_gripper)
    
    
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
    
    
    
    
    
    rospy.spin()