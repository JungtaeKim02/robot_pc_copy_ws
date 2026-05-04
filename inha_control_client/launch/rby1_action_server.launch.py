import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt config 로드 (kinematics.yaml 포함)
    moveit_config = MoveItConfigsBuilder(
        "rby1_v1_1",
        package_name="rby1_inha_moveit"
    ).to_moveit_configs()

    # Action Server 노드 (MoveIt kinematics 파라미터 포함)
    action_server_node = Node(
        package="rby1_control_client",
        executable="rby1_action_server",
        name="rby1_grasp_action_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,  # kinematics.yaml (bio_ik 설정)
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([action_server_node])
