from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import RobotCommander
import rospy

# Initialize node
rospy.init_node('cartesian_trajectory')

# Get the robot's kinematic model
robot = RobotCommander()
group = robot.get_group("manipulator")

# Define Cartesian waypoints
waypoints = [
    [0.002, 0.193, 1.003],  # Initial position
    [0.462, 0.116, 0.467],  # Move up and sideways
]

# IK to get joint angles for waypoints
trajectory = JointTrajectory()
trajectory.joint_names = group.get_joints()

for point in waypoints:
    joint_positions = group.inverse_kinematics(point)
    traj_point = JointTrajectoryPoint()
    traj_point.positions = joint_positions
    traj_point.time_from_start = rospy.Duration(1.0)  # Adjust timing
    trajectory.points.append(traj_point)

# Publish trajectory to controller
trajectory_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory', JointTrajectory, queue_size=1)
trajectory_pub.publish(trajectory)
