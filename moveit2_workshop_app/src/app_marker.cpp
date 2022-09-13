// Copyright (c) Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    ///
    /// ---------------------------- Initialization ------------------------
    ///

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "app_simple",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("app_marker");
    std::this_thread::sleep_for(5000ms); // Wait to let other nodes launch
    RCLCPP_WARN_STREAM(logger, "---------Starting marker application---------");

    // Get parameters. 
    // No need to declare first since NodeOptions is instructed to automatically declare.
    std::string planning_group = node->get_parameter("planning_group").as_string();
    std::string marker1_frame = node->get_parameter("marker1_frame").as_string();
    std::string marker2_frame = node->get_parameter("marker2_frame").as_string();
    RCLCPP_WARN_STREAM(logger, "planning_group: " << planning_group);
    RCLCPP_WARN_STREAM(logger, "marker frames: " << marker1_frame << " " << marker2_frame);

    // Setup moveit
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, planning_group);

    // Setup TF2
    auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    geometry_msgs::msg::TransformStamped transformStamped;
    // The target frame which is used as reference for planning by moveit
    std::string to_frame = move_group_interface.getRobotModel()->getLinkModel(0)->getName();
    std::string from_frame;

    ///
    /// ---------------------------- Marker detection ------------------------
    ///

    // Get marker 1 pose
    from_frame = marker1_frame;
    while(!tf_buffer_->canTransform(to_frame, from_frame, tf2::TimePointZero) && rclcpp::ok())
    {
         RCLCPP_WARN_STREAM(logger, "Waiting for transform from " << from_frame << " to " << to_frame);
         std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(logger, "transform available");
    transformStamped = tf_buffer_->lookupTransform(to_frame, from_frame,tf2::TimePointZero);

    geometry_msgs::msg::Pose target1_pose;
    target1_pose.position.x = transformStamped.transform.translation.x;
    target1_pose.position.y = transformStamped.transform.translation.y;
    target1_pose.position.z = transformStamped.transform.translation.z + 0.5;
    target1_pose.orientation.x = 1.0;
    target1_pose.orientation.y = 0.0;
    target1_pose.orientation.z = 0.0;
    target1_pose.orientation.w = 0.0;

    // Get marker 2 pose
    from_frame = marker2_frame;
    while(!tf_buffer_->canTransform(to_frame, from_frame, tf2::TimePointZero) && rclcpp::ok())
    {
         RCLCPP_WARN_STREAM(logger, "Waiting for transform from " << from_frame << " to " << to_frame);
         std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(logger, "transform available");
    transformStamped = tf_buffer_->lookupTransform(to_frame, from_frame,tf2::TimePointZero);

    geometry_msgs::msg::Pose target2_pose;
    target2_pose.position.x = transformStamped.transform.translation.x;
    target2_pose.position.y = transformStamped.transform.translation.y;
    target2_pose.position.z = transformStamped.transform.translation.z + 0.5;
    target2_pose.orientation.x = 1.0;
    target2_pose.orientation.y = 0.0;
    target2_pose.orientation.z = 0.0;
    target2_pose.orientation.w = 0.0;

    ///
    /// ---------------------------- Planning & Execution ------------------------
    ///
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success;

    // target1 phase
    RCLCPP_WARN_STREAM(logger, "---------Plan target1---------");
    move_group_interface.setPoseTarget(target1_pose);
    success = static_cast<bool>(move_group_interface.plan(plan));

    if (success)
    {
        RCLCPP_WARN_STREAM(logger, "---------Execute target1---------");
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger, "---------Planning failed---------");
    }

    // target2 phase
    RCLCPP_WARN_STREAM(logger, "---------Plan target2---------");
    move_group_interface.setPoseTarget(target2_pose);
    success = static_cast<bool>(move_group_interface.plan(plan));

    if (success)
    {
        RCLCPP_WARN_STREAM(logger, "---------Execute target2---------");
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger, "---------Planning failed---------");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}