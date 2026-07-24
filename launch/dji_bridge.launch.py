"""
dji_bridge.launch.py — Launch the Jetson <-> ROS 2 serial bridge.

dji_serial_bridge_node talks to the MCB over UART, translating between the
DJI-framed protocol and ROS topics for the five message types it knows
about (nav_goal, cv_target, pose, ref_sys, relocalize). It has no
opinion on where those ROS topics' other ends come from -- upstream
producers (sentry_pkg's mcb_relay, the CV pipeline, etc.) publish/subscribe
directly on this node's topics, remapped as needed.

Override parameters from the command line, e.g.:
  ros2 launch dji_serial_bridge dji_bridge.launch.py device:=/dev/ttyUSB0 baudrate:=115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dji_serial_bridge")

    return LaunchDescription([
        DeclareLaunchArgument(
            "device",
            default_value="/dev/ttyTHS1",
            description="Serial device path (e.g. /dev/ttyTHS1 or /dev/ttyUSB0)"),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Serial baud rate in bits-per-second"),
        DeclareLaunchArgument(
            "debug_log",
            default_value="true",
            description="Log everything"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [pkg_share, "config", "dji_bridge_params.yaml"]),
            description="Full path to a ROS 2 parameters YAML file"),

        Node(
            package="dji_serial_bridge",
            executable="dji_serial_bridge_node",
            name="dji_serial_bridge",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "device":   LaunchConfiguration("device"),
                    "baudrate": LaunchConfiguration("baudrate"),
                    "debug_log": LaunchConfiguration("debug_log"),
                },
            ],
        ),
    ])
