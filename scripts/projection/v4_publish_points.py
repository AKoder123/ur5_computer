#! /usr/bin/env python

import numpy as np
import rospy
import tf
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Initialize the ROS node
rospy.init_node('point_finding_node', anonymous=True)

# Function to transform points to robot base frame
def transform_point_to_base(point, camera_to_base_tf):
    point_homogeneous = np.append(point, 1)  # Convert to homogeneous coordinates
    return np.dot(camera_to_base_tf, point_homogeneous)[:3]

# Initialize the Transform Listener for the camera to robot base frame transformation
listener = tf.TransformListener()

# Set up the publisher for the collision point data (float value)
pub = rospy.Publisher('/current_float_value', Float32, queue_size=10)

# Simulation parameters for forward motion
gripper_start = np.array([0, 0, 0])  # Starting gripper position in camera frame
forward_direction = np.array([0, 0, 1])  # Forward in the camera's frame
step_size = 0.01  # Step size in meters

# Global variable for point cloud data
point_cloud = None

# Callback to handle incoming point cloud data
def point_cloud_callback(msg):
    global point_cloud
    point_cloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

# Subscribe to the RealSense point cloud topic
rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback)

rate = rospy.Rate(10)  # Loop at 10 Hz

# Main loop to continuously check for the collision point
while not rospy.is_shutdown():
    # Dynamically update the transformation from camera to robot base frame
    try:
        (trans, rot) = listener.lookupTransform("base_link", "camera_link2", rospy.Time(0))
        camera_to_base_tf = tf.transformations.compose_matrix(
            translate=trans, angles=tf.transformations.euler_from_quaternion(rot)
        )
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform not available. Skipping this iteration.")
        rate.sleep()
        continue

    if point_cloud is None:
        rospy.logwarn("No point cloud data received yet. Waiting...")
        rate.sleep()
        continue

    # Convert point cloud to numpy array
    vertices = np.array(point_cloud)

    # Filter invalid points (remove outliers or points with large Z values)
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
                rospy.loginfo("Contact point in base frame: " + str(contact_point_base))
                contact_found = True
                break

    rate.sleep()  # Control the loop rate
