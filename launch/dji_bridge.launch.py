"""
dji_bridge.launch.py — Launch the Jetson <-> ROS 2 serial bridge, plus the
adapter that feeds it computer-vision targets.

Two nodes are started:
  * dji_serial_bridge_node  — talks to the MCB over UART.
  * point_to_cv_target_node — subscribes to the vision pipeline's
    /roi_point (geometry_msgs/PointStamped, REP-103 camera frame) and
    republishes it as this package's CVTarget message on cv_target_topic,
    which is remapped into dji_serial_bridge_node's "~/cv_target" input.
    Disable it with enable_cv_target_bridge:=false if you intend to publish
    CVTarget yourself.

Override parameters from the command line, e.g.:
  ros2 launch dji_serial_bridge dji_bridge.launch.py device:=/dev/ttyUSB0 baudrate:=115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            "enable_cv_target_bridge",
            default_value="true",
            description="Launch point_to_cv_target_node to feed /roi_point "
                        "into the bridge's cv_target input"),
        DeclareLaunchArgument(
            "roi_point_topic",
            default_value="/roi_point",
            description="PointStamped topic published by "
                        "roi_depth_query/roi_depth_node"),
        DeclareLaunchArgument(
            "roi_topic",
            default_value="/roi",
            description="Detection2D topic read only for a confidence score"),
        DeclareLaunchArgument(
            "cv_target_topic",
            default_value="/cv_target",
            description="Topic carrying CVTarget between the adapter and the bridge"),
        DeclareLaunchArgument(
            "estimate_velocity",
            default_value="true",
            description="Finite-difference v_x/v_y/v_z/a_x/a_y/a_z from "
                        "consecutive /roi_point samples"),

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
            remappings=[
                ("~/cv_target", LaunchConfiguration("cv_target_topic")),
                # Other private topics keep their default
                # /dji_serial_bridge/... names; uncomment to rename them too:
                # ("~/pose",       "/robot/pose"),
                # ("~/ref_sys",    "/robot/ref_sys"),
                # ("~/nav_goal",   "/navigation/goal"),
                # ("~/relocalize", "/localization/position"),
            ],
        ),

        # ── Vision -> CVTarget adapter ────────────────────────────────────────
        # Converts /roi_point (geometry_msgs/PointStamped) into CVTarget and
        # publishes it on cv_target_topic, which the bridge node above is
        # remapped to consume.
        Node(
            package="dji_serial_bridge",
            executable="point_to_cv_target_node",
            name="point_to_cv_target",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_cv_target_bridge")),
            parameters=[{
                "point_topic":       LaunchConfiguration("roi_point_topic"),
                "confidence_topic":  LaunchConfiguration("roi_topic"),
                "output_topic":      LaunchConfiguration("cv_target_topic"),
                "estimate_velocity": LaunchConfiguration("estimate_velocity"),
            }],
        ),
    ])
