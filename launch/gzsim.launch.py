
"""
gzsim.launch.py
"""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import os
from random import randint
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate Launch description with Gazebo11"""
    world_options = {0: 'renault', 1: 'aston_martin', 2: 'redbull'}
    world_path = os.path.join(get_package_share_directory(
        'ros_follow_line'), 'worlds', f'simple_circuit_{world_options[randint(0, 2)]}.world')

    world_path = os.path.join(get_package_share_directory(
        'ros_follow_line'), 'worlds', 'simple_circuit_renault.sdf')
    bridges_file = os.path.join(get_package_share_directory(
        'ros_follow_line'), 'config', 'bridges.yaml')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=['true', 'false'],
            description='Launch in verbose mode.'),
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Path to world file.'),
        DeclareLaunchArgument(
            'config_file',
            default_value=bridges_file,
            description='Configuration file for the bridge'
        ),
        DeclareLaunchArgument(
            'game_logic',
            default_value='true',
            choices=['true', 'false'],
            description='Launch Game Logic.'),
        # Launch processes
        SetParameter(name='use_sim_time', value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[
                {'config_file': LaunchConfiguration('config_file')},
            ]
        ),
    ])
