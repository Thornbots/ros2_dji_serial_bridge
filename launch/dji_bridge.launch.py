"""
dji_bridge.launch.py — Launch the Jetson ↔ ROS 2 serial bridge.

Override parameters from the command line, e.g.:
  ros2 launch dji_serial_bridge dji_bridge.launch.py device:=/dev/ttyUSB0 baudrate:=115200
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dji_serial_bridge")

    return LaunchDescription([
        # ── Overridable arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            "device",
            default_value="/dev/ttyTHS1",
            description="Serial device path (e.g. /dev/ttyTHS1 or /dev/ttyUSB0)"),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Serial baud rate in bits-per-second"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [pkg_share, "config", "dji_bridge_params.yaml"]),
            description="Full path to a ROS 2 parameters YAML file"),

        # ── Bridge node ────────────────────────────────────────────────────────
        Node(
            package="dji_serial_bridge",
            executable="dji_serial_bridge_node",
            name="dji_serial_bridge",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                # Command-line overrides win over the YAML file.
                {
                    "device":   LaunchConfiguration("device"),
                    "baudrate": LaunchConfiguration("baudrate"),
                },
            ],
            # Topic remapping examples — uncomment and adjust as needed:
            # remappings=[
            #     ("~/pose",       "/robot/pose"),
            #     ("~/ref_sys",    "/robot/ref_sys"),
            #     ("~/nav_goal",   "/navigation/goal"),
            #     ("~/cv_target",  "/cv/target"),
            #     ("~/relocalize", "/localization/position"),
            # ],
        ),
    ])
