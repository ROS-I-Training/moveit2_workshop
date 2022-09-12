# Copyright (c) Fraunhofer IPA
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit2_workshop_bringup_dir = get_package_share_directory('moveit2_workshop_bringup')
    # ur5e_cell_dir = get_package_share_directory('ur5e_cell')

    include_marker_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit2_workshop_bringup_dir,'launch', 'marker_detection.launch.py')),
        launch_arguments={'start_rviz': 'False'}.items())

    # TODO: @ipa-rar - include everything required to start robot, moveit & rviz
    # include_cell_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ur5e_cell_dir,'launch', 'full_cell.launch.py')),
    #     launch_arguments={'start_rviz': 'False'}.items())

    # TODO: @ipa-rar and @ipa-kut
    # declare_app_node = Node(
    #     package="moveit2_workshop_app",
    #     executable="moveit2_workshop_app",
    #     output="screen",
    #     name="demo_app_node")

    ld = LaunchDescription()

    ld.add_action(include_marker_detection)
    # TODO: @ipa-rar
    # ld.add_action(include_cell_launch)
    # TODO: @ipa-rar and @ipa-kut
    # ld.add_action(declare_app_node)
    return ld
    