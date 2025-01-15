import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Initialize ROS node and CV Bridge
rospy.init_node("color_image_viewer")
bridge = CvBridge()

# left dot position

dot_x, dot_y = 300, 200

def depth_image_callback(msg):
    try:
        
        # print("here")
        # Convert ROS Image message to OpenCV format (depth image)
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Get the depth value at the red dot location
        depth_value = depth_image[dot_y, dot_x]
        
        # Print depth value in meters (or whatever unit the depth image is in)
        print("Depth at red dot: " + str(depth_value))

    except Exception as e:
        # rospy.logerr(f"Failed to process depth image: {e}")
        pass



def color_image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        color_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        cv2.circle(color_image, (dot_x, dot_y), radius=5, color=(0, 0, 255), thickness=-1)


        # Display the color image
        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)  # Required to refresh the OpenCV window
    except Exception as e:
        # rospy.logerr(f"Failed to process color image: {e}")
        pass

# Subscribe to the color image topic
rospy.Subscriber("/camera/color/image_raw", Image, color_image_callback)
rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)

# Keep the program alive
rospy.spin()