# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
dji_bridge.launch.py — Launch the Jetson <-> ROS 2 serial bridge.

dji_serial_bridge_node translates between the DJI-framed UART protocol and
ROS topics for its five message types (nav_goal, cv_target, pose, ref_sys,
relocalize). It has no opinion on where those topics' other ends come
from. Override parameters from the command line
(e.g. device:=/dev/ttyUSB0); see README.md for a full example.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('dji_serial_bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyTHS1',
            description='Serial device path (e.g. /dev/ttyTHS1 or /dev/ttyUSB0)'),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baud rate in bits-per-second'),
        DeclareLaunchArgument(
            'debug_log',
            default_value='true',
            description='Log everything'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'dji_bridge_params.yaml']),
            description='Full path to a ROS 2 parameters YAML file'),

        Node(
            package='dji_serial_bridge',
            executable='dji_serial_bridge_node',
            name='dji_serial_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'device':   LaunchConfiguration('device'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'debug_log': LaunchConfiguration('debug_log'),
                },
            ],
        ),
    ])
