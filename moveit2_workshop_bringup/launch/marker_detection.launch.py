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
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import yaml
from yaml.loader import SafeLoader

def generate_launch_description():

    start_rviz = LaunchConfiguration('start_rviz')

    declare_arg_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='True',
        description='Whether to start rviz',
        )

    moveit2_workshop_bringup_dir = get_package_share_directory('moveit2_workshop_bringup')

    params_aruco_node = os.path.join(
        moveit2_workshop_bringup_dir,
        'config/marker_detection',
        'aruco_node_params.yaml')

    params_camera = os.path.join(
        moveit2_workshop_bringup_dir,
        'config/marker_detection',
        'camera_node_params.yaml')

    params_camera_tf = os.path.join(
        moveit2_workshop_bringup_dir,
        'config/marker_detection',
        'camera_tf.yaml')

    declare_aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        output="screen",
        parameters = [params_aruco_node])
    
    declare_usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output="screen",
        name="camera_node",
        namespace = "/usb_camera",
        parameters=[params_camera])
    
    declare_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d' + os.path.join(moveit2_workshop_bringup_dir, 'config/marker_detection', 'rviz.rviz')],
        output="screen",
        condition=IfCondition(start_rviz))

    with open(params_camera_tf) as cam_params:
        data = yaml.load(cam_params, Loader=SafeLoader)
        camera_tf = data["camera_tf"]

    declare_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=camera_tf,
        output="screen")
    
    ld = LaunchDescription()

    ld.add_action(declare_arg_rviz)

    ld.add_action(declare_aruco_node)
    ld.add_action(declare_usb_cam_node)
    ld.add_action(declare_rviz_node)
    ld.add_action(declare_camera_tf)
    
    return ld