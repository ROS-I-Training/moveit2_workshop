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

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "app_simple",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("app_simple");
    RCLCPP_WARN_STREAM(logger, "---------Starting simple application---------");

    // Get parameters. 
    // No need to declare first since NodeOptions is instructed to automatically declare.
    std::string planning_group = node->get_parameter("planning_group").as_string();
    std::vector<double> target1_pose_vals = node->get_parameter("target1_pose").as_double_array();
    std::vector<double> target2_pose_vals = node->get_parameter("target2_pose").as_double_array();

    RCLCPP_WARN_STREAM(logger, "---------");
    RCLCPP_WARN_STREAM(logger, "planning_group: " << planning_group);
    RCLCPP_WARN_STREAM(logger, "target1_pose_vals: " 
                                    << target1_pose_vals[0] << " "
                                    << target1_pose_vals[1] << " "
                                    << target1_pose_vals[2] << " "
                                    << target1_pose_vals[3] << " "
                                    << target1_pose_vals[4] << " "
                                    << target1_pose_vals[5] << " "
                                    << target1_pose_vals[6] << " ");

    RCLCPP_WARN_STREAM(logger, "target2_pose_vals: " 
                                    << target2_pose_vals[0] << " "
                                    << target2_pose_vals[1] << " "
                                    << target2_pose_vals[2] << " "
                                    << target2_pose_vals[3] << " "
                                    << target2_pose_vals[4] << " "
                                    << target2_pose_vals[5] << " "
                                    << target2_pose_vals[6] << " ");
    RCLCPP_WARN_STREAM(logger, "---------");

    // Set target poses based on parameters
    geometry_msgs::msg::Pose target1_pose;
    target1_pose.position.x = target1_pose_vals[0];
    target1_pose.position.y = target1_pose_vals[1];
    target1_pose.position.z = target1_pose_vals[2];
    target1_pose.orientation.x = target1_pose_vals[3];
    target1_pose.orientation.y = target1_pose_vals[4];
    target1_pose.orientation.z = target1_pose_vals[5];
    target1_pose.orientation.w = target1_pose_vals[6];

    geometry_msgs::msg::Pose target2_pose;
    target2_pose.position.x = target2_pose_vals[0];
    target2_pose.position.y = target2_pose_vals[1];
    target2_pose.position.z = target2_pose_vals[2];
    target2_pose.orientation.x = target2_pose_vals[3];
    target2_pose.orientation.y = target2_pose_vals[4];
    target2_pose.orientation.z = target2_pose_vals[5];
    target2_pose.orientation.w = target2_pose_vals[6];

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, planning_group);

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