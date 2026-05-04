import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rby1_v1_1", package_name="rby1_inha_moveit").to_moveit_configs()

    move_to_marker_node = Node(
        name="moveit_calibration_node",
        executable="move_to_marker.py",
        package="rby1_inha_moveit",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([move_to_marker_node])

