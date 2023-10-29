/**
 * @file hybrid_planning_interface.hpp
 * @author Balachandra Bhat (bnbhat311@gmail.com)
 * @brief It is an interface to MoveIt 2 Hybrid Planner Action Server.
 * @version 0.1
 * @date 2023-09-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <iostream>
#include <exception>
#include <string>
#include <thread>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <rclcpp/subscription.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "arc_interfaces/srv/arc_target_pose.hpp"

using namespace std::chrono_literals;
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("HPI");
}

class HybridPlanningInterface
{
    public:
        HybridPlanningInterface(const rclcpp::Node::SharedPtr& node);
        bool move(const geometry_msgs::msg::PoseStamped& target_pose);
        geometry_msgs::msg::PoseStamped user_input();
        geometry_msgs::msg::PoseStamped get_pose(float x, float y, float z);
        void init();
        void run();
        moveit_msgs::msg::MotionPlanRequest createMotionPlanRequest();
        moveit_msgs::msg::Constraints createPoseGoal(const geometry_msgs::msg::PoseStamped& target_pose);
        moveit_msgs::msg::MotionSequenceRequest createSequenceRequest(const moveit_msgs::msg::MotionPlanRequest& goal_motion_request);
        rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions createGoalOptions();

    private:
        rclcpp::Node::SharedPtr m_node;
        rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr m_hp_action_client;
        rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr m_global_solution_subscriber;
        planning_scene_monitor::PlanningSceneMonitorPtr m_planning_scene_monitor;
        rclcpp::TimerBase::SharedPtr m_timer;
        std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
        std::string m_planning_group;
        std::shared_ptr<moveit::core::RobotState> m_robot_state;
        rclcpp::Service<arc_interfaces::srv::ArcTargetPose>::SharedPtr m_target_pose_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_cancel_all_goal_service;
        void target_pose_service_callback(
            const std::shared_ptr<arc_interfaces::srv::ArcTargetPose::Request> request,
            std::shared_ptr<arc_interfaces::srv::ArcTargetPose::Response> response);
        void cancel_all_goal_service_callback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};

