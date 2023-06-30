#!/usr/bin/env python3

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    joy_config = LaunchConfiguration("joy_config")
    joy_dev = LaunchConfiguration("joy_dev")
    config_filepath = LaunchConfiguration("config_filepath")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_vel", default_value="/cmd_vel"),
            DeclareLaunchArgument("joy_config", default_value="logitech"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument(
                "config_filepath",
                default_value=[  # ../gamepad/config/teleop.config.yaml
                    TextSubstitution(
                        text=os.path.join(
                            get_package_share_directory("gamepad"), "config", ""
                        )
                    ),
                    joy_config,
                    TextSubstitution(text=".config.yaml"),
                ],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {
                        "dev": joy_dev,
                        "deadzone": 0.1,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[config_filepath],
                remappings={("/velocity_controller/commands", LaunchConfiguration("joy_vel"))},
            ),
            Node(
                package="gamepad",
                executable="gamepad_node",
                name="gamepad_node",
            ),
        ]
    )
