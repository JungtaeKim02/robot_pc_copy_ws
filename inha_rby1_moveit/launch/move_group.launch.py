import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "rby1_v1_1", 
        package_name="rby1_inha_moveit"
    ).sensors_3d(
        file_path=os.path.join(
            get_package_share_directory("rby1_inha_moveit"),
            "config/sensors_3d.yaml"
        )
    ).to_moveit_configs()
    return generate_move_group_launch(moveit_config)
