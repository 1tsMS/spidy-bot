# ROS 2: view the URDF in RViz with joint sliders.  (Not tested yet - no ROS in the sim workflow.)
#   colcon build --packages-select spidy_description && ros2 launch spidy_description display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("spidy_description")
    urdf = xacro.process_file(os.path.join(pkg, "urdf", "spidy.urdf.xacro")).toxml()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
