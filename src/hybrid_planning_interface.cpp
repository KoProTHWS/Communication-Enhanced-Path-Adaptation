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


void HybridPlanningInterface::move(const geometry_msgs::msg::PoseStamped& target_pose)
{

		// Create desired motion goal
		moveit_msgs::msg::MotionPlanRequest goal_motion_request;

		moveit::core::robotStateToRobotStateMsg(*m_robot_state, goal_motion_request.start_state);
		goal_motion_request.group_name = m_planning_group;
		goal_motion_request.num_planning_attempts = 10;
		goal_motion_request.max_velocity_scaling_factor = 0.1;
		goal_motion_request.max_acceleration_scaling_factor = 0.1;
		goal_motion_request.allowed_planning_time = 2.0;
		goal_motion_request.planner_id = "ompl";
		goal_motion_request.pipeline_id = "ompl";
		
		std::vector<double> tolerance_pose(3, 0.01);
		std::vector<double> tolerance_angle(3, 0.01);

		moveit_msgs::msg::Constraints pose_goal =
		kinematic_constraints::constructGoalConstraints("tool0", target_pose, tolerance_pose, tolerance_angle);

		goal_motion_request.group_name = m_planning_group;
		goal_motion_request.goal_constraints.push_back(pose_goal);

		// Create Hybrid Planning action request
		moveit_msgs::msg::MotionSequenceItem sequence_item;
		sequence_item.req = goal_motion_request;
		sequence_item.blend_radius = 0.0;  // Single goal

		moveit_msgs::msg::MotionSequenceRequest sequence_request;
		sequence_request.items.push_back(sequence_item);

		auto goal_action_request = moveit_msgs::action::HybridPlanner::Goal();
		goal_action_request.planning_group = m_planning_group;
		goal_action_request.motion_sequence = sequence_request;

		auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
		send_goal_options.result_callback =
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
		send_goal_options.feedback_callback =
				[](rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr /*unused*/,
					const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback> feedback) {
					RCLCPP_INFO(LOGGER, feedback->feedback.c_str());
				};

		RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
		// Ask server to achieve some goal and wait until it's accepted
		auto goal_handle_future = m_hp_action_client->async_send_goal(goal_action_request, send_goal_options);
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

void HybridPlanningInterface::run()
{
	init();
	try {
		while (true) {
			auto target_pose = get_pose(0.26, 0.5, 0.2);
			move(target_pose);
			rclcpp::sleep_for(5s);
			target_pose = get_pose(0.26, 0.5, 0.3);
			move(target_pose);
			rclcpp::sleep_for(5s);
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(LOGGER, "Exception thrown: %s", e.what());
	}
}