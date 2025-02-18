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
        self.projection_position_pub = rospy.Publisher("/projection_position", Float32, queue_size=10)
        self.projection_scale_pub = rospy.Publisher("/projection_scale", Float32, queue_size=10)

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
        
        screen_width = width
        

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

        

        # Define the orange rectangles
        rect1_top_left = (width // 2 - 90, height // 2 + 10)
        rect1_bottom_right = (width // 2 - 50, height // 2 + 90)
        
        rect2_top_left = (width // 2 + 160, height // 2 + 10)
        rect2_bottom_right = (width // 2 + 200, height // 2 + 90)

        # Draw orange rectangles
        # cv2.rectangle(overlay_image, rect1_top_left, rect1_bottom_right, (0, 165, 255), -1)
        # cv2.rectangle(overlay_image, rect2_top_left, rect2_bottom_right, (0, 165, 255), -1)

        # Calculate centers of the orange rectangles
        rect1_center = (
            (rect1_top_left[0] + rect1_bottom_right[0]) // 2,
            (rect1_top_left[1] + rect1_bottom_right[1]) // 2
        )
        rect2_center = (
            (rect2_top_left[0] + rect2_bottom_right[0]) // 2,
            (rect2_top_left[1] + rect2_bottom_right[1]) // 2
        )

        # Calculate the original gap between the centers of the orange rectangles
        original_gap = rect2_center[0] - rect1_center[0]

        # Extract the depth values for the rectangle centers
        depth_rect1 = self.depth_image[rect1_center[1], rect1_center[0]] * 0.001  # Convert to meters
        depth_rect2 = self.depth_image[rect2_center[1], rect2_center[0]] * 0.001  # Convert to meters

        # Calculate the inverse scaling factor
        scale_factor = max(depth_rect1, depth_rect2) / depth_in_meters if depth_in_meters > 0 else 1.0
        scale_factor = pow(scale_factor, -1)

        # Scale the gap and dimensions
        scaled_gap = int(original_gap / scale_factor)

        # Calculate new centers for the purple rectangles
        purple_rect1_center_x = center_x - scaled_gap // 2 + 20  
        purple_rect2_center_x = center_x + scaled_gap // 2+ 15

        # Scale the dimensions of the purple rectangles
        def scale_rectangle(top_left, bottom_right, scale_factor):
            center_x = (top_left[0] + bottom_right[0]) // 2
            center_y = (top_left[1] + bottom_right[1]) // 2
            half_width = int((bottom_right[0] - top_left[0]) / scale_factor / 2)
            half_height = int((bottom_right[1] - top_left[1]) / scale_factor / 2)
            new_top_left = (center_x - half_width, center_y - half_height)
            new_bottom_right = (center_x + half_width, center_y + half_height)
            return new_top_left, new_bottom_right

        rect1_top_left_purple, rect1_bottom_right_purple = scale_rectangle(
            (purple_rect1_center_x - 20, rect1_center[1] - 40 - 35),
            (purple_rect1_center_x + 20, rect1_center[1] + 40 - 35),
            scale_factor
        )
        rect2_top_left_purple, rect2_bottom_right_purple = scale_rectangle(
            (purple_rect2_center_x - 20, rect2_center[1] - 40 - 35),
            (purple_rect2_center_x + 20, rect2_center[1] + 40 - 35),
            scale_factor
        )
        
        pos_value = purple_rect2_center_x / float(screen_width)
        
        self.projection_position_pub.publish(pos_value)
        self.projection_scale_pub.publish(scale_factor)
        
        circle_size = 50
        
        # Draw the green dot at the contact point
        cv2.circle(overlay_image, (center_x+10, center_y), int(circle_size/(scale_factor/2)), (0, 255, 0), -1)

        # Draw purple rectangles
        # cv2.rectangle(overlay_image, rect1_top_left_purple, rect1_bottom_right_purple, (255, 0, 255), -1)
        # cv2.rectangle(overlay_image, rect2_top_left_purple, rect2_bottom_right_purple, (255, 0, 255), -1)
        
        

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
