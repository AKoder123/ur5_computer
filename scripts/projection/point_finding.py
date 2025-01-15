import pyrealsense2 as rs
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped

# Initialize the ROS node
rospy.init_node('point_finding_node', anonymous=True)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Only depth stream
pipeline.start(config)

# Function to transform points to robot base frame
def transform_point_to_base(point, camera_to_base_tf):
    point_homogeneous = np.append(point, 1)  # Convert to homogeneous coordinates
    return np.dot(camera_to_base_tf, point_homogeneous)[:3]

# Get depth frame and generate point cloud
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
if not depth_frame:
    exit()

# Generate point cloud from depth frame
pc = rs.pointcloud()
points = pc.calculate(depth_frame)  # Generate the point cloud
vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # Extract vertices

# Load transformation from camera to robot base frame
listener = tf.TransformListener()
listener.waitForTransform("base_link", "camera_link2", rospy.Time(), rospy.Duration(4.0))
(trans, rot) = listener.lookupTransform("base_link", "camera_link2", rospy.Time())
camera_to_base_tf = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

# Simulate forward motion
gripper_start = np.array([0, 0, 0])  # Replace with the gripper's position in the camera frame
forward_direction = np.array([0, 0, 1])  # Forward in the camera's frame
step_size = 0.01  # Step size in meters

for step in range(100):  # Move 1 meter forward (100 steps)
    current_position = gripper_start + step * step_size * forward_direction
    # Check if this point intersects the point cloud
    for point in vertices:
        distance = np.linalg.norm(current_position - point)
        if distance < 0.01:  # Threshold for contact
            contact_point_base = transform_point_to_base(point, camera_to_base_tf)
            print("Contact point in base frame: " + str(contact_point_base[2]))
            break
