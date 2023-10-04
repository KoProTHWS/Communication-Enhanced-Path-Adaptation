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

	m_target_pose_service = m_node->create_service<arc_interfaces::srv::ArcTargetPose>("move_to_target_pose", 
        std::bind(&HybridPlanningInterface::target_pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));

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
    
    auto send_goal_options = createGoalOptions();
    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    auto goal_handle_future = m_hp_action_client->async_send_goal(goal_action_request, send_goal_options);

	// Get the goal handle from the future. This will block until the goal has been sent.
	//auto goal_handle = goal_handle_future.get();
	//if (!goal_handle) {
	//	RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
	//	return false;
	//}

	// Wait for the server to be done with the goal
	//auto result_future = m_hp_action_client->async_get_result(goal_handle);

	return true;
}

moveit_msgs::msg::MotionPlanRequest HybridPlanningInterface::createMotionPlanRequest() 
{
    moveit_msgs::msg::MotionPlanRequest request;
    moveit::core::robotStateToRobotStateMsg(*m_robot_state, request.start_state);
    request.group_name = m_planning_group;
    request.num_planning_attempts = 10;
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

rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions HybridPlanningInterface::createGoalOptions() {
    auto options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
    // Result callback setup
    options.result_callback =
			[](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
					switch (result.code)
					{
						case rclcpp_action::ResultCode::SUCCEEDED:
							RCLCPP_INFO(LOGGER, "Hybrid planning goal succeeded");
							break;
						case rclcpp_action::ResultCode::ABORTED:
							RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
							return;
						case rclcpp_action::ResultCode::CANCELED:
							RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
							return;
						default:
							RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
							return;
							RCLCPP_INFO(LOGGER, "Hybrid planning result received");
					}
				};

    options.feedback_callback =
				[](rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr /*unused*/,
					const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback> feedback) {
					RCLCPP_INFO(LOGGER, "Hybrid planning feedback received");
					RCLCPP_INFO(LOGGER, feedback->feedback.c_str());
				};

    return options;
}

geometry_msgs::msg::PoseStamped HybridPlanningInterface::user_input()
{
    float x, y, z;
    std::cout << "Enter target pose (or 'c' to cancel): " << std::endl;
    std::cout << "Enter x: ";
    std::cin >> x;
    std::cout << "Enter y: ";
    std::cin >> y;
    std::cout << "Enter z: ";
    std::cin >> z;

    // Check for cancellation
    if (std::cin.fail()) {
        throw std::runtime_error("User canceled input.");
    }

    std::cout << "Moving to - x: " << x << " y: " << y << " z: " << z << std::endl;
    std::cout << "Press 'c' to cancel or enter to continue: ";

    // Check for user cancellation while waiting for input
    char choice;
    std::cin.ignore(); // Ignore any leftover characters in the input buffer
    std::cin.get(choice);

    if (choice == 'c') {
        throw std::runtime_error("User canceled move.");
    }

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

void HybridPlanningInterface::run()
{
	try {
		while (true) {
			auto target_pose = get_pose(0.26, 0.5, 0.2);
			move(target_pose);
			rclcpp::sleep_for(5s);
			target_pose = get_pose(0.26, 0.5, 0.45);
			move(target_pose);
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(LOGGER, "Exception thrown: %s", e.what());
	}
}
