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
from tf.transformations import euler_from_matrix, euler_matrix, quaternion_from_matrix
def apply_rpy_in_ee_frame_to_wpose(wpose, delta_roll, delta_pitch, delta_yaw):
    try:
        # Current orientation of wpose (base_link frame)
        current_orientation = wpose.orientation
        current_quat = [
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w,
        ]
        current_rotation_matrix = tf.transformations.quaternion_matrix(current_quat)

        # Delta rotation in EE frame
        ee_delta_rotation = tf.transformations.euler_matrix(
            delta_roll, delta_pitch, delta_yaw, axes='rxyz'
        )

        # Apply the delta rotation to the current rotation matrix (pre-multiply)
        new_rotation_matrix = np.dot(ee_delta_rotation, current_rotation_matrix)

        # Convert the updated rotation matrix back to a quaternion
        new_quat = tf.transformations.quaternion_from_matrix(new_rotation_matrix)

        # Update wpose with the new quaternion
        wpose.orientation.x = new_quat[0]
        wpose.orientation.y = new_quat[1]
        wpose.orientation.z = new_quat[2]
        wpose.orientation.w = new_quat[3]

        return wpose

    except Exception as e:
       # rospy.logerr(f"Error applying RPY in EE frame to wpose: {e}")
        return None




if __name__ == "__main__":
    rospy.init_node('apply_rpy_changes')
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander('manipulator')

    # Get current pose
    wpose = move_group.get_current_pose().pose

    # Example: Apply roll, pitch, yaw changes in end-effector frame
    delta_roll = np.deg2rad(10)   # Convert degrees to radians if necessary
    delta_pitch = 0
    delta_yaw = 0

    updated_wpose = apply_rpy_in_ee_frame_to_wpose(wpose, delta_roll, delta_pitch, delta_yaw)

    # Plan and execute with updated wpose
    waypoints = [copy.deepcopy(updated_wpose)]
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    raw_input("Check RViz for plan. Press Enter to execute...")
    move_group.execute(plan, wait=True)

    print(updated_wpose)
    rospy.spin()
