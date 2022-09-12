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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    include_marker_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("moveit2_workshop_bringup"), "/launch", "/marker_detection.launch.py"]),
        launch_arguments={'start_rviz': 'False'}.items(),)

    include_cell_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur5e_cell_bringup"), "/launch", "/ur5e_cell_bringup.launch.py"]),)

    # TODO: @ipa-rar and @ipa-kut
    # declare_app_node = Node(
    #     package="moveit2_workshop_app",
    #     executable="moveit2_workshop_app",
    #     output="screen",
    #     name="demo_app_node")

    ld = LaunchDescription()

    ld.add_action(include_marker_detection)
    ld.add_action(include_cell_launch)
    # TODO: @ipa-rar and @ipa-kut
    # ld.add_action(declare_app_node)
    return ld
