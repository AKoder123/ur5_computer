#!/usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Global variable for point cloud data
point_cloud_data = None

# Callback to handle incoming point cloud data
def point_cloud_callback(msg):
    global point_cloud_data
    # Convert ROS PointCloud2 message to a numpy array
    point_cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

# Initialize ROS node
rospy.init_node('point_cloud_visualizer', anonymous=True)

# Subscribe to the RealSense point cloud topic
rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback)

def visualize_point_cloud():
    print("Press CTRL+C to stop visualization...")
    
    # Open3D visualizer setup
    vis = o3d.visualization.Visualizer()
    vis.create_window("ROS Point Cloud Visualization")
    o3d_point_cloud = o3d.geometry.PointCloud()

    try:
        while not rospy.is_shutdown():
            if point_cloud_data:
                # Convert point cloud data to numpy array
                np_point_cloud = np.array(point_cloud_data, dtype=np.float32)

                # Filter invalid points (e.g., out-of-range or NaN values)
                valid_points = np_point_cloud[(~np.isnan(np_point_cloud).any(axis=1)) & (np_point_cloud[:, 2] < 2.0)]

                # Update Open3D point cloud object
                o3d_point_cloud.points = o3d.utility.Vector3dVector(valid_points)

                # Update visualization
                vis.add_geometry(o3d_point_cloud)
                vis.update_geometry(o3d_point_cloud)
                vis.poll_events()
                vis.update_renderer()

    except rospy.ROSInterruptException:
        print("ROS Interrupt detected, shutting down visualization...")
    finally:
        vis.destroy_window()

if __name__ == "__main__":
    visualize_point_cloud()