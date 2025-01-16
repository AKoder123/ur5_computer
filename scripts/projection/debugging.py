#!/usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

point_cloud_data = None

def point_cloud_callback(msg):
    global point_cloud_data
    points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    point_cloud_data = points_list

def visualize_point_cloud():
    print("Press CTRL+C to stop visualization...")
    
    # Create Visualizer window
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ROS Point Cloud Visualization")

    # Create an empty PointCloud object (we'll update it in the loop)
    o3d_point_cloud = o3d.geometry.PointCloud()
    vis.add_geometry(o3d_point_cloud)

    # Set background to black & show coordinate frame
    render_option = vis.get_render_option()
    render_option.background_color = np.array([0.0, 0.0, 0.0])
    render_option.show_coordinate_frame = True

    try:
        while not rospy.is_shutdown():
            # Only update if we have data
            if point_cloud_data is not None and len(point_cloud_data) > 0:
                np_point_cloud = np.array(point_cloud_data, dtype=np.float32)
                valid_points = np_point_cloud[~np.isnan(np_point_cloud).any(axis=1)]

                if valid_points.shape[0] > 0:
                    # --- 1. Capture current camera parameters ---
                    view_ctl = vis.get_view_control()
                    cam_params = view_ctl.convert_to_pinhole_camera_parameters()

                    # Remove old geometry from the scene
                    vis.remove_geometry(o3d_point_cloud, reset_bounding_box=False)

                    # Update the points in the PointCloud
                    o3d_point_cloud.points = o3d.utility.Vector3dVector(valid_points)

                    # Optionally set some uniform color
                    colors = np.tile([1.0, 1.0, 1.0], (valid_points.shape[0], 1))
                    o3d_point_cloud.colors = o3d.utility.Vector3dVector(colors)

                    # Re-add the geometry
                    vis.add_geometry(o3d_point_cloud)

           
                    view_ctl.convert_from_pinhole_camera_parameters(cam_params)

            vis.poll_events()
            vis.update_renderer()
            # Small sleep to avoid hogging CPU
            time.sleep(0.05)

    except rospy.ROSInterruptException:
        print("ROS Interrupt detected, shutting down visualization...")
    finally:
        vis.destroy_window()

if __name__ == "__main__":
    rospy.init_node('point_cloud_visualizer', anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback)
    visualize_point_cloud()
