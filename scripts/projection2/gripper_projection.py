#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthReader:
    def __init__(self):
        self.bridge = CvBridge()

        # Publisher for the contact point depth
        self.contact_point_pub = rospy.Publisher("/contact_point", Float32, queue_size=10)

        # Subscribe to the raw depth image
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw", 
            Image, 
            self.depth_callback
        )

        # Subscribe to the color image
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self.color_callback
        )

        self.depth_image = None
        self.color_image = None

    def color_callback(self, color_msg):
        """Callback to store and process the color image."""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: " + str(e))

    def depth_callback(self, depth_msg):
        """Callback to process depth and visualize gripper projections."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: " + str(e))
            return

        if self.color_image is None:
            return  # Skip until we have a color image

        # Get image dimensions and define the contact pixel
        height, width = self.depth_image.shape[:2]
        center_x = width // 2 + 20
        center_y = height // 2 + 7

        # Depth at the green dot (center)
        depth_value = self.depth_image[center_y, center_x]
        depth_in_meters = depth_value * 0.001  # Assuming depth is in mm

        # Publish the depth value and log it
        self.contact_point_pub.publish(depth_in_meters)
        print("Depth at contact point (" 
              + str(center_x) 
              + ", " 
              + str(center_y) 
              + "): " 
              + "{:.3f}".format(depth_in_meters) 
              + " m")

        # Make a copy of the color image for overlay
        overlay_image = self.color_image.copy()

        # Draw the green dot at the contact point
        cv2.circle(overlay_image, (center_x, center_y), 5, (0, 255, 0), -1)

        # Define the orange rectangles
        rect1_top_left = (width // 2 - 90, height // 2 + 10)
        rect1_bottom_right = (width // 2 - 50, height // 2 + 90)
        rect2_top_left = (width // 2 + 160, height // 2 + 10)
        rect2_bottom_right = (width // 2 + 200, height // 2 + 90)

        # Draw orange rectangles
        cv2.rectangle(overlay_image, rect1_top_left, rect1_bottom_right, (0, 165, 255), -1)
        cv2.rectangle(overlay_image, rect2_top_left, rect2_bottom_right, (0, 165, 255), -1)

        # Calculate centers of the orange rectangles
        rect1_center = (
            (rect1_top_left[0] + rect1_bottom_right[0]) // 2,
            (rect1_top_left[1] + rect1_bottom_right[1]) // 2
        )
        rect2_center = (
            (rect2_top_left[0] + rect2_bottom_right[0]) // 2,
            (rect2_top_left[1] + rect2_bottom_right[1]) // 2
        )

        # Extract and print the depth values for the rectangle centers
        depth_rect1 = self.depth_image[rect1_center[1], rect1_center[0]]
        depth_rect2 = self.depth_image[rect2_center[1], rect2_center[0]]

        depth_rect1_m = depth_rect1 * 0.001  # Convert mm to m
        depth_rect2_m = depth_rect2 * 0.001  # Convert mm to m

        print("Depth at rect1 center (" 
              + str(rect1_center[0]) 
              + ", " 
              + str(rect1_center[1]) 
              + "): " 
              + "{:.3f}".format(depth_rect1_m) 
              + " m")
        print("Depth at rect2 center (" 
              + str(rect2_center[0]) 
              + ", " 
              + str(rect2_center[1]) 
              + "): " 
              + "{:.3f}".format(depth_rect2_m) 
              + " m")

        # Visualize the centers of the orange rectangles
        cv2.circle(overlay_image, rect1_center, 5, (255, 255, 255), -1)
        cv2.circle(overlay_image, rect2_center, 5, (255, 255, 255), -1)

        # Show the final image
        cv2.imshow("Gripper Projection Visualization", overlay_image)

        # ESC to exit
        key = cv2.waitKey(1)
        if key == 27:
            rospy.signal_shutdown("User exited.")

if __name__ == "__main__":
    rospy.init_node("depth_reader_node", anonymous=True)
    reader = DepthReader()
    rospy.spin()
    cv2.destroyAllWindows()
