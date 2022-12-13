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

 
#define _USE_MATH_DEFINES
 
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "app_simple",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.  ##This solved get cuurent pose issue**
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    auto const logger = rclcpp::get_logger("app_simple");

    // Get parameters.
    // No need to declare first since NodeOptions is instructed to automatically declare.
    std::string planning_group = node->get_parameter("planning_group").as_string();
    std::vector<double> target1_pose_vals = node->get_parameter("target1_pose").as_double_array();
    std::vector<double> target2_pose_vals = node->get_parameter("target2_pose").as_double_array();

    moveit::core::VariableBounds bx, by, bz;
    bx.position_bounded_ = by.position_bounded_ = bz.position_bounded_ = true;

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

    RCLCPP_INFO(node->get_logger(), "Pose 1: %s", geometry_msgs::msg::to_yaml(target2_pose).c_str());

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;

    auto move_group_interface = MoveGroupInterface(node, planning_group);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group_interface.setStartStateToCurrentState();
    // move_group_interface.setPlanningPipelineId("ompl");
    // move_group_interface.setPlannerId("RRTstarkConfigDefault");
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP");

    auto current_pose = move_group_interface.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current Pose: %s", geometry_msgs::msg::to_yaml(current_pose).c_str());
    RCLCPP_INFO(node->get_logger(), "Target Pose 1: %s", geometry_msgs::msg::to_yaml(target1_pose).c_str());
    move_group_interface.setPoseTarget(target1_pose);
    move_group_interface.allowReplanning(true);
    move_group_interface.setNumPlanningAttempts(10);
    move_group_interface.setPlanningTime(5.0);

    auto state = move_group_interface.getCurrentState();
    auto success = move_group_interface.plan(plan);

    if (success.val == 1)
    {
        RCLCPP_INFO(node->get_logger(), "Successfully generated motion plan. Executing...");
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Failed generating motion plan. Exiting...");
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Current Pose: %s", geometry_msgs::msg::to_yaml(current_pose).c_str());
    RCLCPP_INFO(node->get_logger(), "Target Pose 2: %s", geometry_msgs::msg::to_yaml(target2_pose).c_str());

    move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(target2_pose);
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("LIN");

    success = move_group_interface.plan(plan);

    if (success.val == 1)
    {
        RCLCPP_INFO(node->get_logger(), "Successfully generated motion plan. Executing...");
        move_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Failed generating motion plan. Exiting...");
        rclcpp::shutdown();
        return 0;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}