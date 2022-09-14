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

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    moveit2_workshop_bringup_dir = get_package_share_directory('moveit2_workshop_bringup')

    params_dummy_app_marker_node = os.path.join(
        moveit2_workshop_bringup_dir,
        'config/app/panda',
        'app_marker.yaml')

    declare_app_node = Node(
        package="moveit2_workshop_app",
        executable="app_marker",
        output="screen",
        parameters = [params_dummy_app_marker_node]
        )

    marker1_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.5", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "marker_1"],
    )

    marker2_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["-0.5", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "marker_2"],
    )

    include_panda_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit2_workshop_bringup_dir,'launch/includes', 'panda_demo.launch.py')))
    ld = LaunchDescription()

    ld.add_action(marker1_tf_node)
    ld.add_action(marker2_tf_node)
    ld.add_action(include_panda_demo)
    ld.add_action(declare_app_node)

    return ld
