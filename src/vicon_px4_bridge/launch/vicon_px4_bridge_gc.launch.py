from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("vicon_px4_bridge"), "config", "vicon_px4_params.yaml"]
        ),
        description="Path to the configuration YAML file",
    )

    config_file = LaunchConfiguration("config_file")

    nodes = []

    NUM_DRONES = 3

    for drone_id in range(NUM_DRONES):
        drone_name = f"multilift_{drone_id}"

        vicon_topic_name = f"/vrpn_mocap/{drone_name}/pose"

        px4_topic_prefix = "" if drone_id == 0 else f"/px4_{drone_id}"
        px4_topic_name = f"{px4_topic_prefix}/fmu/in/vehicle_visual_odometry"

        nodes.append(
            Node(
                package="vicon_px4_bridge",
                executable="vicon_px4_bridge_node",
                name=f"vicon_px4_bridge_{drone_id}",
                output="screen",
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                    {
                        "vicon_topic_name": vicon_topic_name,
                        "px4_topic_name": px4_topic_name,
                        "drone_id": drone_id,
                        "drone_name": drone_name,
                    },
                ],
            )
        )

    return LaunchDescription(
        [
            config_file_arg,
            *nodes,
        ]
    )
