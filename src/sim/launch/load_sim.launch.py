import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory("sim")

    # Set model path
    gazebo_model_path = os.path.join(pkg_sim, "models")

    # Set environment variables - added GZ_VERBOSE for plugin logging
    env = {
        "GAZEBO_MODEL_PATH": gazebo_model_path,
        "GZ_VERBOSE": "3",  # Show all log levels (debug, info, warning, error)
    }
    env_string = ";".join([f"{key}={value}" for key, value in env.items()])

    # Gazebo Harmonic launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r ", os.path.join(pkg_sim, "worlds", "track_layout2.world")],
            # Pass all environment variables
            "env_vars": env_string,
        }.items(),
    )

    # Bridge between ROS2 and Gazebo Harmonic
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_sim, "config", "bridge_config.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Spawn robot using gz service
    spawn_car = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ackermann_car",
            "-file",
            os.path.join(pkg_sim, "models", "ackermann_car", "model.sdf"),
            "-x",
            "12.0",
            "-y",
            "11",
            "-z",
            "0.5",
            "-Y",
            "3.14",
        ],
        output="screen",
    )

    return LaunchDescription([gz_sim, bridge, spawn_car])
