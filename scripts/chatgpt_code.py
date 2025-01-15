import rospy
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from tf2_geometry_msgs import do_transform_twist

def transform_twist_to_tool_frame(twist_msg, source_frame, target_frame):
    # Initialize TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # Look up the transform from source to target frame
        
        
        return twist_transformed
    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform error: {e}")
        return None

if __name__ == "__main__":
    rospy.init_node("twist_command_transformer")
    pub = rospy.Publisher("/twist_controller/command", Twist, queue_size=10)
    
    twist_base = Twist()
    twist_base.linear.x = 0.1  # 0.1 m/s forward
    twist_base.angular.z = 0.1  # 0.1 rad/s rotation

    while not rospy.is_shutdown():
        # Transform Twist command to the tool frame
        twist_tool = transform_twist_to_tool_frame(twist_base, "base_link", "tool0")
        if twist_tool:
            pub.publish(twist_tool)
        rospy.sleep(0.1)
