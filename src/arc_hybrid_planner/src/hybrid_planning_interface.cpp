#include "arc_hybrid_planner/hybrid_planning_interface.hpp"

HybridPlanningInterface::HybridPlanningInterface(const rclcpp::Node::SharedPtr& node)
: m_node(node), m_planning_group("ur_manipulator")
{	
	std::string hybrid_planning_action_name = "";
	if (m_node->has_parameter("hybrid_planning_action_name"))
	{
		m_node->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
	}
	else
	{
		RCLCPP_ERROR(LOGGER, "hybrid_planning_action_name parameter was not defined");
		std::exit(EXIT_FAILURE);
	}
	m_hp_action_client = rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(m_node, hybrid_planning_action_name);

	m_target_pose_service = m_node->create_service<arc_interfaces::srv::ArcTargetPose>("/arc_planning_interface/move_to_target_pose", 
        std::bind(&HybridPlanningInterface::target_pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    m_cancel_all_goal_service = m_node->create_service<std_srvs::srv::Trigger>("/arc_planning_interface/cancel_all_goals", 
        std::bind(&HybridPlanningInterface::cancel_all_goal_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    m_hp_comm_publisher = m_node->create_publisher<arc_interfaces::msg::ArcHPComm>("/arc_planning_interface/hp_comm", 1);
}

void HybridPlanningInterface::init()
{
    if (!m_hp_action_client->wait_for_action_server(20s))
    {
        RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
        return;
    }

    robot_model_loader::RobotModelLoader robot_model_loader(m_node, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    m_robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = m_robot_state->getJointModelGroup(m_planning_group);

    // Configure a valid robot state
    m_robot_state->setToDefaultValues(joint_model_group, "ready");
    m_robot_state->update();
}


bool HybridPlanningInterface::move(const geometry_msgs::msg::PoseStamped& target_pose)
{
	moveit_msgs::msg::MotionPlanRequest goal_motion_request = createMotionPlanRequest();
    goal_motion_request.goal_constraints.push_back(createPoseGoal(target_pose));
    
    moveit_msgs::msg::MotionSequenceRequest sequence_request = createSequenceRequest(goal_motion_request);
    auto goal_action_request = moveit_msgs::action::HybridPlanner::Goal();
    goal_action_request.planning_group = m_planning_group;
    goal_action_request.motion_sequence = sequence_request;

    goal_motion_request.max_acceleration_scaling_factor = 0.01;
    goal_motion_request.max_velocity_scaling_factor = 0.01;

    goal_motion_request.max_cartesian_speed = 0.1;
    
    auto send_goal_options = createGoalOptions();
    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    auto goal_handle_future = m_hp_action_client->async_send_goal(goal_action_request, send_goal_options);
    return true;
}

bool HybridPlanningInterface::move_blocking(const geometry_msgs::msg::PoseStamped& target_pose)
{
	moveit_msgs::msg::MotionPlanRequest goal_motion_request = createMotionPlanRequest();
    goal_motion_request.goal_constraints.push_back(createPoseGoal(target_pose));
    
    moveit_msgs::msg::MotionSequenceRequest sequence_request = createSequenceRequest(goal_motion_request);
    auto goal_action_request = moveit_msgs::action::HybridPlanner::Goal();
    goal_action_request.planning_group = m_planning_group;
    goal_action_request.motion_sequence = sequence_request;

    goal_motion_request.max_acceleration_scaling_factor = 0.01;
    goal_motion_request.max_velocity_scaling_factor = 0.01;

    // reduce speed
    goal_motion_request.max_cartesian_speed = 0.1;
    
    auto send_goal_options = createGoalOptions();
    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    auto goal_handle_future = m_hp_action_client->async_send_goal(goal_action_request, send_goal_options);

    return true;
}


moveit_msgs::msg::MotionPlanRequest HybridPlanningInterface::createMotionPlanRequest() 
{
    moveit_msgs::msg::MotionPlanRequest request;
    moveit::core::robotStateToRobotStateMsg(*m_robot_state, request.start_state);
    request.group_name = m_planning_group;
    request.num_planning_attempts = 100;
    request.max_velocity_scaling_factor = 0.1;
    request.max_acceleration_scaling_factor = 0.1;
    request.allowed_planning_time = 2.0;
    request.planner_id = "ompl";
    request.pipeline_id = "ompl";
    return request;
}

moveit_msgs::msg::Constraints HybridPlanningInterface::createPoseGoal(const geometry_msgs::msg::PoseStamped& target_pose) 
{
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    return kinematic_constraints::constructGoalConstraints("tool0", target_pose, tolerance_pose, tolerance_angle);
}

moveit_msgs::msg::MotionSequenceRequest HybridPlanningInterface::createSequenceRequest(const moveit_msgs::msg::MotionPlanRequest& goal_motion_request) {
    moveit_msgs::msg::MotionSequenceItem sequence_item;
    sequence_item.req = goal_motion_request;
    sequence_item.blend_radius = 0.0;
    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(sequence_item);
    return sequence_request;
}

void HybridPlanningInterface::hp_action_result_callback(const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
    arc_interfaces::msg::ArcHPComm hp_comm_msg;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(LOGGER, "Hybrid planning goal succeeded");
            is_planning_success = 1;
            hp_comm_msg.status = 1; // 1 for success, 0 for failure
            hp_comm_msg.message = "Hybrid planning goal succeeded";
            hp_comm_msg.time_stamp = m_node->now();
            m_hp_comm_publisher->publish(hp_comm_msg);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
            is_planning_success = -1; // -1 for failure 1 for success 0 for not started
            hp_comm_msg.status = 0; // 1 for success, 0 for failure
            hp_comm_msg.message = "Hybrid planning goal aborted";
            hp_comm_msg.time_stamp = m_node->now();
            m_hp_comm_publisher->publish(hp_comm_msg);
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
            return;
        case rclcpp_action::ResultCode::UNKNOWN:
            RCLCPP_ERROR(LOGGER, "Hybrid planning goal failed");
            return;
        default:
            RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
            return;
    }
    RCLCPP_INFO(LOGGER, "Hybrid planning result received");
}

rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions HybridPlanningInterface::createGoalOptions() {
    auto options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
    options.result_callback = std::bind(&HybridPlanningInterface::hp_action_result_callback, this, std::placeholders::_1);
    options.feedback_callback =
				[](rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr /*unused*/,
					const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback> feedback) {
					//RCLCPP_INFO(LOGGER, "Hybrid planning feedback received");
					//RCLCPP_INFO(LOGGER, feedback->feedback.c_str());
				};

    return options;
}

geometry_msgs::msg::PoseStamped HybridPlanningInterface::get_pose(float x, float y, float z)
{
	geometry_msgs::msg::PoseStamped target_pose;
	target_pose.header.frame_id = "world";
	target_pose.pose.position.x = x;
	target_pose.pose.position.y = y;
	target_pose.pose.position.z = z;
	target_pose.pose.orientation.x = 1.0;
	target_pose.pose.orientation.y = 0.0;
	target_pose.pose.orientation.z = 0.0;
	target_pose.pose.orientation.w = 0.0;

	return target_pose;
}

void HybridPlanningInterface::target_pose_service_callback(
    const std::shared_ptr<arc_interfaces::srv::ArcTargetPose::Request> request,
    std::shared_ptr<arc_interfaces::srv::ArcTargetPose::Response> response)
{   
    RCLCPP_INFO(LOGGER, "Received target pose request");
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.pose = request->pose;
    // Assuming "world" as the frame of reference. Modify if needed.
    target_pose.header.frame_id = "world";

    response->success = move(target_pose);
}

void HybridPlanningInterface::cancel_all_goal_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(LOGGER, "Received cancel all goal request");
    m_hp_action_client->async_cancel_all_goals();
    response->success = true;
}

void HybridPlanningInterface::run()
{
	try {
		while (true) {
			auto target_pose = get_pose(0.17, 0.5, 0.3);
			move(target_pose);
			rclcpp::sleep_for(2s);
			target_pose = get_pose(-0.4, 0.5, 0.3);
			move(target_pose);
            rclcpp::sleep_for(2s);
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(LOGGER, "Exception thrown: %s", e.what());
	}
}
