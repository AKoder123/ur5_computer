#! /usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import rospy
import tf
from std_msgs.msg import Float32
from time import time

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
pub = rospy.Publisher('/current_float_value', Float32, queue_size=10)

# Simulation parameters for forward motion
gripper_start = np.array([0, 0, 0])  # Starting gripper position in camera frame
forward_direction = np.array([0, 0, 1])  # Forward in the camera's frame
step_size = 0.01  # Step size in meters
rate = rospy.Rate(10)  # Loop at 10 Hz

# Main loop to continuously check for the collision point
while not rospy.is_shutdown():
    # Get the latest point cloud data
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        rate.sleep()
        continue  # If no depth frame, skip this iteration

    # Generate point cloud from depth frame
    pc = rs.pointcloud()
    points = pc.calculate(depth_frame)
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # Extract vertices

    # Filter invalid points (remove outliers or points with large Z values)
    vertices = vertices[~np.isnan(vertices).any(axis=1)]
    vertices = vertices[vertices[:, 2] < 2.0]  # Filter points beyond 2 meters

    # Simulate forward motion by updating gripper position
    contact_found = False
    for step in range(100):  # Simulate moving forward in steps
        current_position = gripper_start + step * step_size * forward_direction

        # Check for collision points within the point cloud
        distances = np.linalg.norm(vertices - current_position, axis=1)
        close_points = vertices[distances < 0.01]  # Points close to the gripper

        if len(close_points) > 0:
            closest_point = close_points[0]
            contact_point_base = transform_point_to_base(closest_point, camera_to_base_tf)

            # Publish only the first contact point
            if not contact_found:
                pub.publish(contact_point_base[2])
                print("Contact point in base frame: " + str(contact_point_base))
                contact_found = True
                break

    rate.sleep()  # Control the loop rate
