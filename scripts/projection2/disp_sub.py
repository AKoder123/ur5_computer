#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthReader:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribe to the raw depth image
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw", 
            Image, 
            self.depth_callback
        )

        # Optionally, subscribe to camera info for intrinsics if needed
        self.camera_info_sub = rospy.Subscriber(
            "/camera/depth/camera_info", 
            CameraInfo, 
            self.info_callback
        )

        self.camera_model = None

    def info_callback(self, msg):
        """
        Store camera intrinsics here if you need to deproject
        the pixel to 3D coordinates.
        """
        self.camera_model = msg  # or parse into a camera model library

    def depth_callback(self, depth_msg):
        try:
            # Convert the ROS Image message to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: " + str(e))
            return

        # depth_image is a 2D array, each pixel is the depth in millimeters or meters
        height, width = depth_image.shape[:2]
        center_x = width // 2
        center_y = height // 2

        # Get the raw depth value at the center pixel
        depth_value = depth_image[center_y, center_x]

        # If depth is in millimeters, convert to meters as needed
        # (It depends on your driver configuration. Some publish in mm, some in tenths of mm, etc.)
        # For RealSense default, it's usually in mm.
        depth_in_meters = depth_value * 0.001

        # Logging the depth using string concatenation
        rospy.loginfo("Depth at pixel (" + str(center_x) + ", " + str(center_y) + "): " +
                      "{:.3f}".format(depth_in_meters) + " m")

if __name__ == "__main__":
    rospy.init_node("depth_reader_node", anonymous=True)
    reader = DepthReader()
    rospy.spin()
