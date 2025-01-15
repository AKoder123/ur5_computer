#! /usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import moveit_commander
import open3d
from time import sleep
import roslib
roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import Constraints, JointConstraint


# Initialize the ROS node
rospy.init_node('point_finding_node', anonymous=True)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Only depth stream
pipeline.start(config)

# Function to transform points to robot base frame
def transform_point_to_base(point, camera_to_base_tf):
    point_homogeneous = np.append(point, 1)  # Convert to homogeneous coordinates
    return np.dot(camera_to_base_tf, point_homogeneous)[:3]

# Initialize the Transform Listener for the camera to robot base frame transformation
listener = tf.TransformListener()
listener.waitForTransform("base_link", "camera_link2", rospy.Time(), rospy.Duration(4.0))
(trans, rot) = listener.lookupTransform("base_link", "camera_link2", rospy.Time())
camera_to_base_tf = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

# Set up the publisher for the collision point data (float value)
pub = rospy.Publisher('/current_float_value', Float32, queue_size=2)

# Simulation parameters for forward motion
gripper_start = np.array([0, 0, 0])  # Starting gripper position in camera frame
forward_direction = np.array([0, 0, 1])  # Forward in the camera's frame
step_size = 0.01  # Step size in meters
count = 0

# Main loop to continuously check for the collision point
while not rospy.is_shutdown():
    # Get the latest point cloud data
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        continue  # If no depth frame, skip this iteration
    
    # Generate point cloud from depth frame
    pc = rs.pointcloud()
    points = pc.calculate(depth_frame)
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # Extract vertices
    
    # Simulate forward motion by updating gripper position
    for step in range(100):  # Example of moving the gripper forward in steps
        current_position = gripper_start + step * step_size * forward_direction
        
        # Check for collision points within the point cloud
        for point in vertices:
            distance = np.linalg.norm(current_position - point)
            if distance < 0.01:  # Threshold for contact
                contact_point_base = transform_point_to_base(point, camera_to_base_tf)
                
                # Publish the Z-coordinate of the contact point in the base frame
                pub.publish(contact_point_base[2])
                rospy.sleep(0.1)  # Sleep to control loop rate
                
                print("Contact point in base frame: " + str(contact_point_base))
                count += 1
                break  # Exit the loop if a contact point is found

    rospy.sleep(0.5)  # Sleep to control the loop rate for the next cycle
