#include "behavior_tree_executor/behavior_tree_executor.hpp"

#include "bt_moveit2_mtc_nodes/bt_moveit2_nodes.hpp"
#include "bt_moveit2_mtc_nodes/connect_node.hpp"
#include "bt_service_interfaces/srv/execute_mtc_task.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <moveit/task_constructor/stage.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
namespace behavior_tree_executor {

BehaviorTreeExecutor::BehaviorTreeExecutor(rclcpp::NodeOptions options)
    : rclcpp::Node("behavior_tree_executor", options) {
    
    update_bt_srv_ = this->create_service<bt_service_interfaces::srv::UpdateBTXml>(
            "update_bt_xml",
            std::bind(&BehaviorTreeExecutor::onUpdateBTXml, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

    execute_mtc_task_srv_ = this->create_service<bt_service_interfaces::srv::ExecuteMtcTask>(
            "execute_mtc_task",
            std::bind(&BehaviorTreeExecutor::onExecuteMtcTask, this,
                      std::placeholders::_1, std::placeholders::_2));

}

void BehaviorTreeExecutor::onExecuteMtcTask(
    const std::shared_ptr<bt_service_interfaces::srv::ExecuteMtcTask::Request> request,
    std::shared_ptr<bt_service_interfaces::srv::ExecuteMtcTask::Response> response)
{
    (void)request; // 请求为空

    if (!factory_ || !main_tree_) {
      response->success = false;
      response->message = "No tree loaded. Please send tree xml to service /update_bt_xml to load tree first.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    
    try {
        RCLCPP_INFO(get_logger(), "start to plan");
        // 规划任务
        if (!task_->plan(5)) {
            response->success = false;
            response->message = "Planning failed";
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
        }

        // 可选，将规划发布到rviz
        RCLCPP_INFO(get_logger(), "start to push task");
        task_->introspection().publishSolution(*task_->solutions().front());

        // 执行任务
        RCLCPP_INFO(get_logger(), "start to execute task");
        auto result = task_->execute(*task_->solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            response->success = false;
            response->message = "Planning failed";
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
        }

        response->success = true;
        response->message = "Planning and executed succeeded!";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const moveit::task_constructor::InitStageException& e) {
        response->success = false;
        response->message = "Exception during execution: " + std::string(e.what());
        RCLCPP_ERROR_STREAM(get_logger(), e);
    }
}

void BehaviorTreeExecutor::onUpdateBTXml(
    const std::shared_ptr<bt_service_interfaces::srv::UpdateBTXml::Request> req,
    std::shared_ptr<bt_service_interfaces::srv::UpdateBTXml::Response> res)
{
    std::lock_guard<std::mutex> lock(bt_mutex_);

    try {
        
        initTree(req->xml);
        auto status = tickBTNode();

        // 初始化任务
        RCLCPP_INFO(get_logger(), "start to init task");
        task_->init();

        res->success = true;
        res->message = "ok";
        RCLCPP_INFO(this->get_logger(), "Behavior tree updated from service, xml=%s, tree ticked NodeStatus=%d",
                                                                    req->xml.c_str(), static_cast<int>(status));
    } catch (const moveit::task_constructor::InitStageException& e) {
        res->success = false;
        res->message = std::string("Error to load BT XML, result=") + e.what();
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
        return;
    }
}

BT::NodeStatus BehaviorTreeExecutor::tickBTNode() {
    std::lock_guard<std::mutex> lock(tick_mutex_);
    return main_tree_->tickOnce();
}

void BehaviorTreeExecutor::initTask(moveit_mtc_bt_parameters::Params params) {
    RCLCPP_INFO_STREAM(this->get_logger(), "resetTask");
    if (!task_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "initTask");
        task_ = std::make_shared<moveit::task_constructor::Task>();
        task_->stages()->setName("bt mtc task");
        task_->loadRobotModel(this->shared_from_this());

        // Set task properties
        task_->setProperty("group", params.arm_group_name);
        task_->setProperty("eef", params.eef_name);
        task_->setProperty("hand", params.hand_group_name);
        task_->setProperty("hand_grasping_frame", params.hand_frame);
        task_->setProperty("ik_frame", params.hand_frame);
        return;
    }

    task_->reset();
}

void BehaviorTreeExecutor::init() {
    RCLCPP_INFO_STREAM(this->get_logger(), "init behavior tree executor");
    // Get the Shared Pointer from the node
    auto node = this->shared_from_this();

    // start param listener
    RCLCPP_INFO_STREAM(this->get_logger(), "start param listener");
    param_listener_ = std::make_shared<moveit_mtc_bt_parameters::ParamListener>(node);
    auto params = param_listener_->get_params();

    // Create Factory
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    initTask(params);

    // Register Nodes
    factory_->registerNodeType<moveit_mtc_bt_nodes::MoveToBTNode>("MoveToBTNode", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::CurrentStateNode>("CurrentStateNode", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::MoveRelativeBTNode>("MoveRelativeBTNode", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::GenerateGraspPoseBTNode>("GenerateGraspPoseBTNode", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::ModifyPlanningSceneBTNode>("ModifyPlanningSceneBTNode", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::ConnectNode>("ConnectNode", node, task_, params);
}

void BehaviorTreeExecutor::initTree(const std::string& xml) {
    RCLCPP_INFO_STREAM(this->get_logger(), "load tree from new xml");

    if (main_tree_) {
        main_tree_.reset();
    }
    try {
        main_tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml));
    } catch (const BT::BehaviorTreeException& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load tree from XML: " << e.what());
        throw moveit::task_constructor::InitStageException();
    }

    if (groot_logger_) {
        groot_logger_.reset();
    }
    groot_logger_ = std::make_shared<BT::Groot2Publisher>(*main_tree_);

    // Lightweight serialization
    // file_logger_ = std::make_shared<BT::FileLogger2>(*main_tree_,
    // "test.btlog");
    if (cout_logger_) {
        cout_logger_.reset();
    }
    cout_logger_ = std::make_shared<BT::StdCoutLogger>(*main_tree_);

    auto params = param_listener_->get_params();
    initTask(params);
}

} // namespace behavior_tree_executor