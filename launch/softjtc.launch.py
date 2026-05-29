import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def log_level(level: str) -> List[str]:
    return ["--ros-args", "--log-level", level]


def generate_launch_description():
    ld = LaunchDescription()

    description_package_path = get_package_share_directory("roberto_description")
    hardware_package_path = get_package_share_directory("roberto_hardware_interface")

    urdf_path = os.path.join(hardware_package_path, "urdf", "roberto.urdf.xacro")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_package_path, "launch", "display.launch.py")
            ),
            launch_arguments=[("urdf_file", urdf_path)],
        )
    )

    config_path = os.path.join(hardware_package_path, "config", "controllers.yaml")

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[config_path],
            remappings=[("~/robot_description", "/robot_description")],
            arguments=[*log_level("info")],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "softjoint_state_broadcaster",
                "softjoint_trajectory_controller",
                "--controller-manager",
                "/controller_manager",
                *log_level("info"),
            ],
        )
    )

    return ld
