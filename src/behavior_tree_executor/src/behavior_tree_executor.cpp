#include "behavior_tree_executor/behavior_tree_executor.hpp"

#include <behaviortree_cpp/tree_node.h>
#include <functional>

#include "bt_moveit2_mtc_nodes/bt_moveit2_nodes.hpp"
#include "bt_moveit2_mtc_nodes/connect_node.hpp"
#include "bt_service_interfaces/srv/execute_mtc_task.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/stage.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

namespace behavior_tree_executor {

const std::unordered_map<BT::NodeStatus, std::string> BehaviorTreeExecutor::STATUS_MAP{
    {BT::NodeStatus::SUCCESS, "SUCCESS"},
    {BT::NodeStatus::FAILURE, "FAILURE"},
    {BT::NodeStatus::RUNNING, "RUNNING"},
    {BT::NodeStatus::IDLE, "IDLE"},
    {BT::NodeStatus::SKIPPED, "SKIPPED"}
};

BehaviorTreeExecutor::BehaviorTreeExecutor(rclcpp::NodeOptions options)
    : rclcpp::Node("behavior_tree_executor", options) {
    
    update_bt_srv_ = this->create_service<bt_service_interfaces::srv::UpdateBTXml>(
            "update_mtc_bt_xml",
            std::bind(&BehaviorTreeExecutor::onUpdateMtcBTXml, this,
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

void BehaviorTreeExecutor::onUpdateMtcBTXml(
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
        RCLCPP_INFO(this->get_logger(), "Behavior tree updated from service, tree ticked NodeStatus=%s",
                                                                    STATUS_MAP.find(status)->second.c_str());
        RCLCPP_INFO(this->get_logger(), "Behavior tree xml=%s,", req->xml.c_str());
    } catch (const moveit::task_constructor::InitStageException& e) {
        res->success = false;
        res->message = std::string("Error to load BT XML, result=") + e.what();
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
    }
}

BT::NodeStatus BehaviorTreeExecutor::tickBTNode() {
    std::lock_guard<std::mutex> lock(tick_mutex_);
    return main_tree_->tickOnce();
}

void BehaviorTreeExecutor::initTask(moveit_mtc_bt_parameters::Params params) {
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

    task_->clear();
}

void BehaviorTreeExecutor::init() {
    RCLCPP_INFO_STREAM(this->get_logger(), "init behavior tree executor");
    // Get the Shared Pointer from the node
    auto node = this->shared_from_this();

    // start param listener
    RCLCPP_INFO_STREAM(this->get_logger(), "start param listener");
    param_listener_ = std::make_shared<moveit_mtc_bt_parameters::ParamListener>(node);
    auto params = param_listener_->get_params();

    // init planner factory
    moveit_mtc_bt_nodes::PlannerFactory::getInstance().init(node, params);

    // Create Factory
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    initTask(params);

    // Register Nodes
    // Action Nodes
    factory_->registerNodeType<moveit_mtc_bt_nodes::MoveToBTNode>("MoveTo", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::CurrentStateBTNode>("CurrentState", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::MoveRelativeBTNode>("MoveRelative", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::GenerateGraspPoseBTNode>("GenerateGraspPose", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::GeneratePlacePoseBTNode>("GeneratePlacePose", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::ModifyPlanningSceneBTNode>("ModifyPlanningScene", node, task_, params);
    factory_->registerNodeType<moveit_mtc_bt_nodes::ConnectBTNode>("Connect", node, task_, params);

    // moveit action node
    factory_->registerNodeType<moveit_mtc_bt_nodes::AddObjectBTNode>("AddObject", node, task_, params);

    // wrapper
    factory_->registerNodeType<moveit_mtc_bt_nodes::ComputeIKDecorator>("ComputeIK", node, task_, params);

    // container
    factory_->registerNodeType<moveit_mtc_bt_nodes::MTCSerialContainerNode>("SerialContainer", node, task_, params);
}

void BehaviorTreeExecutor::initTree(const std::string& xml) {
    RCLCPP_INFO_STREAM(this->get_logger(), "load tree from new xml");

    if (main_tree_) {
        main_tree_.reset();
    }
    try {
        main_tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml));
        node_relationship_uid_map_.clear();
        buildMap(main_tree_->rootNode());
        auto visitor = [this](BT::TreeNode* node){
            if (auto mtc_node = dynamic_cast<moveit_mtc_bt_nodes::MTCActionBaseNode*>(node)) {
                mtc_node->init(node_relationship_uid_map_);
            } else if (auto mtc_node = dynamic_cast<moveit_mtc_bt_nodes::MTCWrapperBaseNode*>(node)) {
                mtc_node->init(node_relationship_uid_map_);
            } else if (auto mtc_node = dynamic_cast<moveit_mtc_bt_nodes::MTCSerialContainerNode*>(node)) {
                mtc_node->init(node_relationship_uid_map_);
            }
        };
        main_tree_->applyVisitor(visitor);

    } catch (const BT::BehaviorTreeException& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load tree from XML: " << e.what());
        throw moveit::task_constructor::InitStageException();
    }

    if (groot_logger_) {
        groot_logger_.reset();
    }
    groot_logger_ = std::make_shared<BT::Groot2Publisher>(*main_tree_);

    if (cout_logger_) {
        cout_logger_.reset();
    }
    cout_logger_ = std::make_shared<BT::StdCoutLogger>(*main_tree_);

    auto params = param_listener_->get_params();
    initTask(params);
}

void BehaviorTreeExecutor::buildMap(BT::TreeNode* node, BT::TreeNode* parent) {
    // 保存父节点为MTCSerialContainerNode和MTCWrapperDecorator的uid映射，child.uid->parent.uid
    auto control = dynamic_cast<moveit_mtc_bt_nodes::MTCSerialContainerNode*>(parent);
    auto decorator = dynamic_cast<moveit_mtc_bt_nodes::MTCWrapperBaseNode*>(parent);
    if (parent && (control || decorator)) {
        RCLCPP_INFO(get_logger(), "control child name=%s uid=%d, parent name=%s uid=%d",
                    node->name().c_str(), node->UID(), parent->name().c_str(), parent->UID());
        node_relationship_uid_map_[node->UID()] = parent->UID();
    } 
    
    // recursively add children
    if (auto controlNode = dynamic_cast<BT::ControlNode*>(node)) {
        for (auto& child : controlNode->children()) {
            buildMap(child, node);
        }
    } else if (auto decoratorNode = dynamic_cast<BT::DecoratorNode*>(node)) {
        // decorator node only has one child
        buildMap(decoratorNode->child(), node);
    }
}

} // namespace behavior_tree_executor