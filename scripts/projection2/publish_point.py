#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
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
            # Convert the ROS Image message to a NumPy array
            self.color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: " + str(e))

    def depth_callback(self, depth_msg):
        """Callback to process the depth image, display the color image with a marker, and publish depth."""
        try:
            # Convert the ROS Image message to a NumPy array
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: " + str(e))
            return

        if self.color_image is None:
            return  # Skip processing until we have a color image

        # Get the center pixel for depth and color overlay
        height, width = self.depth_image.shape[:2]
        center_x = width // 2 + 20
        center_y = height // 2 + 7

        # Get the raw depth value at the center pixel
        depth_value = self.depth_image[center_y, center_x]

        # Convert depth to meters if needed (assuming mm by default)
        depth_in_meters = depth_value * 0.001

        # Publish the depth value to the /contact_point topic
        self.contact_point_pub.publish(depth_in_meters)

        # Log the depth information
        rospy.loginfo("Depth at pixel (" + str(center_x) + ", " + str(center_y) + "): " +
                      "{:.3f}".format(depth_in_meters) + " m")

        # Overlay a dot on the color image
        overlay_image = self.color_image.copy()
        cv2.circle(overlay_image, (center_x, center_y), 5, (0, 255, 0), -1)

        # Display the color image with the marker
        cv2.imshow("Color Image with Depth Marker", overlay_image)

        # Press ESC to exit
        key = cv2.waitKey(1)
        if key == 27:  # Escape key
            rospy.signal_shutdown("User exited.")

if __name__ == "__main__":
    rospy.init_node("depth_reader_node", anonymous=True)
    reader = DepthReader()
    rospy.spin()
    cv2.destroyAllWindows()
