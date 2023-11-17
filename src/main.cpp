#include "arc_hybrid_planner/hybrid_planning_interface.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hybrid_planning_test_node", "", node_options);

    HybridPlanningInterface HPI(node);
    HPI.init();
    //std::thread run_HPI([&HPI]() {
    //    HPI.run();
    //});
    //HPI.run();
    rclcpp::spin(node);
    //run_HPI.join();
    rclcpp::shutdown();
    return 0;
}