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
        # cv2.circle(overlay_image, (center_x+10, center_y), 5, (0, 255, 0), -1)
        #0.08
        
        # cv2.circle(overlay_image, (center_x+190, center_y+50), 5, (0, 0, 255), -1)
        
        
        # cv2.circle(overlay_image, (center_x+24, center_y), 5, (0, 0, 255), -1)
        #0.45m
        
        add_value = -448.649 * float(depth_in_meters) + 225.892
        
        cv2.circle(overlay_image, (center_x+int(add_value), center_y), 5, (0, 0, 255), -1)
        
        
        
        

        
        

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
