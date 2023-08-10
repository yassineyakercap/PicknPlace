# Copyright (c) 2020 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOG_LEVEL = 'warn'

def generate_launch_description():

    ld = LaunchDescription()

    package_path = get_package_share_directory('picknplace')
    launch_dir = os.path.join(package_path, 'launch')
    robot_config_path = get_package_share_directory('robot_config')
    robot_config_launch_dir = os.path.join(robot_config_path, 'launch')
    
    launch_stack = LaunchConfiguration('launch_stack')
    declare_launch_stack = DeclareLaunchArgument('launch_stack', default_value='true',
                                         description='Enable/Disable Nav2 launch')
    ld.add_action(declare_launch_stack)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true', description='Use simulator time'
    )
    ld.add_action(declare_use_sim_time)

    # Gazebo Environment Launch
    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_config_launch_dir, 'gazebo.launch.py')),
        launch_arguments={ 'use_sim_time': use_sim_time,
                           'world': os.path.join(
                                        package_path,
                                        'worlds',
                                        'no_roof_small_warehouse',
                                        'no_roof_small_warehouse.world',
                                    )
                          }.items()
                        )
    ld.add_action(gazebo_launch_cmd)

    amr_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_config_launch_dir, 'amr.launch.py')),
        launch_arguments={ 'amr_name': 'amr1',
                           'x_pos': '-0.1',
                           'y_pos': '-0.3',
                           'yaw': '3.14159',
                           'use_sim_time': use_sim_time,
                           'launch_stack': launch_stack,
                          }.items()
                        )

    ld.add_action(amr_launch_cmd)

   

    return ld

