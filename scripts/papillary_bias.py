#!/usr/bin/env python3

import rospy
from papillarray_ros_v2.msg import SensorState  # Adjust if the message type is different
from papillarray_ros_v2.srv import BiasRequest  # Corrected service type
import time
THRESHOLD = 4  # Define threshold for near-zero values
COOLDOWN = 1  # Cooldown period in seconds

last_call_time = rospy.Time(0)  # Store last service call time

def sensor_callback(msg):
    global last_call_time

    # Extract gfX, gfY, gfZ values
    gfx = msg.gfX
    gfy = msg.gfY
    gfz = msg.gfZ

    # Ensure the absolute values are within the threshold
    if (abs(gfx) < THRESHOLD) and (abs(gfy) < THRESHOLD) and (abs(gfz) < THRESHOLD):
        current_time = rospy.Time.now()

        # Prevent frequent calls using cooldown
        if (current_time - last_call_time).to_sec() > COOLDOWN:
            rospy.loginfo("Forces are near zero, calling bias service...")
            call_bias_service()
            time.sleep(0.1)
            call_bias_service()
            time.sleep(0.1)
            call_bias_service()
            last_call_time = current_time
        else:
            rospy.loginfo("Skipping bias request - Cooldown active")

def call_bias_service():
    rospy.wait_for_service('/hub_0/send_bias_request')
    try:
        send_bias = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)  # Corrected service name
        response = send_bias()  # Call the service
        rospy.loginfo("Bias request sent successfully.")
    except rospy.ServiceException as e:
        # rospy.logwarn(f"Service call failed: {e}")# Log a warning instead of suppressing errors
        pass

def listener():
    rospy.init_node('bias_caller_node', anonymous=True)
    rospy.sleep(2)  # Initial delay to prevent immediate bias request on startup
    rospy.Subscriber('/hub_0/sensor_0', SensorState, sensor_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
