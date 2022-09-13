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
from moveit_configs_utils import MoveItConfigsBuilder

import yaml
from yaml.loader import SafeLoader

def generate_launch_description():

    moveit2_workshop_bringup_dir = get_package_share_directory('moveit2_workshop_bringup')
    panda_config_dir  = get_package_share_directory('moveit_resources_panda_moveit_config')

    params_dummy_app_simple_node = os.path.join(
        moveit2_workshop_bringup_dir,
        'config/app',
        'dummy_app_simple.yaml')

    rviz_config = os.path.join(
        moveit2_workshop_bringup_dir,
        'config/app',
        'dummy_rviz.rviz')

    declare_app_node = Node(
        package="moveit2_workshop_app",
        executable="app_simple",
        output="screen",
        parameters = [params_dummy_app_simple_node])
    
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path=os.path.join(panda_config_dir, "config","panda.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(panda_config_dir, "config","panda.srdf"))
        .trajectory_execution(file_path=os.path.join(panda_config_dir, "config","gripper_moveit_controllers.yaml"))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ])

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    ld = LaunchDescription()

    ld.add_action(declare_app_node)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)
    ld.add_action(static_tf_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(panda_arm_controller_spawner)
    
    return ld
