import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


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
        'config',
        'aruco_node.yaml')

    params_camera = os.path.join(
        moveit2_workshop_bringup_dir,
        'config',
        'camera_params.yaml')

    declare_aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        output="screen",
        parameters = [params_aruco_node])
    
    declare_usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output="screen",
        name="usb_camera",
        namespace = "/usb_camera",
        parameters=[params_camera])
    
    declare_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d' + os.path.join(moveit2_workshop_bringup_dir, 'config', 'rviz.rviz')],
        output="screen",
        condition=IfCondition(start_rviz))

    declare_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1","0","0","0","0","0","world","usb_camera_link"],
        output="screen")
    
    ld = LaunchDescription()

    ld.add_action(declare_arg_rviz)

    ld.add_action(declare_aruco_node)
    ld.add_action(declare_usb_cam_node)
    ld.add_action(declare_rviz_node)
    ld.add_action(declare_camera_tf)
    
    return ld