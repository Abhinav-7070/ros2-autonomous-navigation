
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.expanduser(
        "~/ros2_ws/src/navigation/src/nav_maps/config"
    )
    state_file = os.path.expanduser(
        "~/ros2_ws/src/navigation/src/nav_maps/maps/saved_map_main.pbstream"
    )

    return LaunchDescription([
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", "turtlebot3_lds_2d.lua",
                "-load_state_filename", state_file,
                "-start_trajectory_with_default_topics", "true",
            ],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])

