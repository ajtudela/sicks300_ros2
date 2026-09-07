#!/usr/bin/env python3
# Copyright  (c) 2024 Alberto J. Tudela Roldán
# Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

"""Launches a Sick S300 laser scanner node and a filter node."""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    # Default filenames and where to find them
    sicks300_dir = get_package_share_directory('sicks300_ros2')
    default_param_file = os.path.join(sicks300_dir, 'params', 'default.yaml')

    # Create the launch configuration variables:
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # Map these variables to arguments: can be set from the command line or a default will be used
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    params_file_launch_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Full path to the Sicks300 parameter file to use'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Prepare the sicks300_ros2 node.
    sicks300_ros2_node = LifecycleNode(
        package='sicks300_ros2',
        namespace=namespace,
        executable='sicks300_ros2',
        name='laser_front',
        parameters=[params_file],
        autostart=True,
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['laser_front:=', LaunchConfiguration('log-level')]]
    )

    laser_filter_node = Node(
        package='laser_filters',
        namespace='',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        remappings=[
            ('/scan_filtered', '/scan/filtered')],
        arguments=[
            '--ros-args',
            '--log-level', ['scan_filter:=', LaunchConfiguration('log-level')]]
    )

    return LaunchDescription([
        declare_namespace_arg,
        params_file_launch_arg,
        declare_log_level_arg,
        sicks300_ros2_node,
        laser_filter_node
    ])
