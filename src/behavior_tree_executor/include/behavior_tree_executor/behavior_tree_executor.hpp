#pragma once
// C Header
#include <eigen3/Eigen/Dense>

// Standard CPP
#include <memory>
#include <string>
#include <unordered_map>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "rcpputils/visibility_control.hpp"

// Behavior Tree
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"

// Messages
#include "builtin_interfaces/msg/time.hpp"
#include "bt_service_interfaces/srv/update_bt_xml.hpp"
#include "bt_service_interfaces/srv/execute_mtc_task.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include "moveit/task_constructor/task.h"

namespace behavior_tree_executor {

class BehaviorTreeExecutor : public rclcpp::Node {

public:
    RCPPUTILS_PUBLIC BehaviorTreeExecutor(rclcpp::NodeOptions options);
    ~BehaviorTreeExecutor() = default;
    
    /**
    * @brief
    *
    */
    void init();

private:

    void onUpdateMtcBTXml(
        const std::shared_ptr<bt_service_interfaces::srv::UpdateBTXml::Request> req,
        std::shared_ptr<bt_service_interfaces::srv::UpdateBTXml::Response> res);

    void onExecuteMtcTask(
        const std::shared_ptr<bt_service_interfaces::srv::ExecuteMtcTask::Request> req,
        std::shared_ptr<bt_service_interfaces::srv::ExecuteMtcTask::Response> res);

    BT::NodeStatus tickBTNode();

    void initTree(const std::string& xml);

    void initTask(moveit_mtc_bt_parameters::Params params);

    void buildMap(BT::TreeNode* node, BT::TreeNode* parent = nullptr);

    // 添加参数变化回调函数
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    std::shared_ptr<BT::Tree> main_tree_;
    std::shared_ptr<BT::Groot2Publisher> groot_logger_;
    std::shared_ptr<BT::FileLogger2> file_logger_;
    std::shared_ptr<BT::StdCoutLogger> cout_logger_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    std::shared_ptr<moveit_mtc_bt_parameters::ParamListener> param_listener_;
    moveit_mtc_bt_parameters::Params params_;

    // 添加参数订阅器
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_subscription_;

    std::unordered_map<uint16_t, uint16_t> node_relationship_uid_map_;

    rclcpp::Service<bt_service_interfaces::srv::UpdateBTXml>::SharedPtr update_bt_srv_;
    rclcpp::Service<bt_service_interfaces::srv::ExecuteMtcTask>::SharedPtr execute_mtc_task_srv_;

    std::mutex bt_mutex_;
    std::mutex tick_mutex_;

    static const std::unordered_map<BT::NodeStatus, std::string> STATUS_MAP;

};
}  // namespace behavior_tree_executor