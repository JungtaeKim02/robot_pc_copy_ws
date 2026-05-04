import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path



def generate_launch_description():
    # Build MoveIt config for rby1_v1_1 (robot name in URDF)
    moveit_config = MoveItConfigsBuilder(
        "rby1_v1_1", 
        package_name="rby1_inha_moveit"
    ).sensors_3d(file_path=os.path.join(get_package_share_directory("rby1_inha_moveit"),"config/sensors_3d.yaml")).to_moveit_configs()
    
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument("db", default_value="false"))
    ld.add_action(DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # Static virtual joint TFs (if exists)
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Move Group
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # MoveIt RViz
    ld.add_action(
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # Warehouse DB (optional)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    return ld
