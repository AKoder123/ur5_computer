import rospy
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('cartesian_control', anonymous=True)

# Publisher for script commands
script_pub = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)

# Create URScript command
command = String()
command.data = 'movel(p[0.462, 0.116, 0.467, -3.088, 0.001, -1.422], a=0.05, v=0.02)'

# Publish command
rate = rospy.Rate(10)
for _ in range(5):
    script_pub.publish(command)
    rate.sleep()

print("Cartesian move command sent!")
