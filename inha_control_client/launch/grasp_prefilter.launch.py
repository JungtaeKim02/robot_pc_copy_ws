"""Launch file for grasp_prefilter_node.

이 노드는 robot_description 파라미터를 사용하여 RobotModel 을 로드한다.
robot_state_publisher 가 이미 /robot_description 을 발행하고 있다는
전제로, 본 launch 는 별도로 robot_description 을 주입하지 않는다.
필요 시(예: 단독 실행) MoveItConfigsBuilder 를 통해 robot_description 을
함께 넘기는 형태로 확장 가능하다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt config: robot_description / SRDF / kinematics
    moveit_config = MoveItConfigsBuilder(
        "rby1_v1_1",
        package_name="rby1_inha_moveit",
    ).to_moveit_configs()

    enable_rigid_block_arg = DeclareLaunchArgument(
        "enable_rigid_block",
        default_value="true",
        description="Enable rigid-block octomap pre-filter.",
    )
    enable_camera_check_arg = DeclareLaunchArgument(
        "enable_camera_check",
        default_value="true",
        description="Enable camera collision check inside the rigid-block step.",
    )
    downstream_timeout_arg = DeclareLaunchArgument(
        "downstream_timeout_sec",
        default_value="120.0",
        description="Timeout for downstream action_server call [s].",
    )

    prefilter_node = Node(
        package="rby1_control_client",
        executable="grasp_prefilter_node",
        name="grasp_prefilter_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {
                "input_service_name": "/execute_grasp_service",
                # 기본은 cuRobo 직결. MoveIt fallback 실험 시 override.
                "downstream_service_name": "/execute_grasp_planning",
                "planning_scene_service": "/get_planning_scene",
                "enable_rigid_block": LaunchConfiguration("enable_rigid_block"),
                "enable_camera_check": LaunchConfiguration("enable_camera_check"),
                "downstream_timeout_sec": LaunchConfiguration("downstream_timeout_sec"),
            },
        ],
    )

    return LaunchDescription([
        enable_rigid_block_arg,
        enable_camera_check_arg,
        downstream_timeout_arg,
        prefilter_node,
    ])
