#include "behavior_tree_executor/behavior_tree_executor.hpp"
#include <rclcpp/logger.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.automatically_declare_parameters_from_overrides(true);

    // Create Nodes
    auto node = std::make_shared<behavior_tree_executor::BehaviorTreeExecutor>(options);
    node->init();

    // Create Executor and Spin
    RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "create SingleThreadedExecutor");
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
