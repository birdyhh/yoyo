#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <memory>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class MTCWrapperBaseNode : public BT::DecoratorNode {
public:
    MTCWrapperBaseNode(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<rclcpp::Node> nh,
                        const std::shared_ptr<moveit::task_constructor::Task> task,
                        const moveit_mtc_bt_parameters::Params& params)
        : BT::DecoratorNode(name, config), node_(nh), task_(task), params_(params) {}

    void init(const std::unordered_map<uint16_t, uint16_t>& uid_map) {
        parent_uid_map_ = uid_map;
    }

protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;
    std::unique_ptr<moveit::task_constructor::WrapperBase> wrapper_;
    std::unordered_map<uint16_t, uint16_t> parent_uid_map_;


protected:
    virtual std::unique_ptr<moveit::task_constructor::WrapperBase> createWrapper() = 0;

    virtual std::string getWrapperType() = 0;


private:

    virtual BT::NodeStatus tick() override {
        // 装饰器节点不直接执行，而是包装其子节点
        // 在MTC中，Wrapper通常包装一个Stage
        wrapper_ = createWrapper();

        // 在执行子节点前，将wrapper引用存储到黑板上，key的命名为前缀+UID
        std::string blackboard_key = WRAPPER_PTR_PREFIX + std::to_string(BT::TreeNode::UID());
        config().blackboard->set(blackboard_key, (moveit::task_constructor::WrapperBase*)wrapper_.get());
        RCLCPP_INFO(node_->get_logger(), "Adding %s wrapper %s to blackboard as %s", getWrapperType().c_str(), name().c_str(), blackboard_key.c_str());

        // 执行子节点
        BT::NodeStatus status = child_node_->executeTick();

        // 子节点执行成功后，将容器添加到任务中
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Adding %s to %s wrapper", child_node_->name().c_str(), name().c_str());
            addChildToParent(std::move(wrapper_));
        }

        return BT::NodeStatus::SUCCESS;
    }

    void addChildToParent(std::unique_ptr<moveit::task_constructor::Stage>&& stage) {
        uint16_t child_uid = BT::TreeNode::UID();
        const auto & parent_uid_it = parent_uid_map_.find(child_uid);
        if (parent_uid_it != parent_uid_map_.end()) {
            std::string parent_uid = std::to_string(parent_uid_it->second);
            std::string container_bb_key = CONTAINER_PTR_PREFIX + parent_uid;
            auto blackboard = config().blackboard;
            // 查找是否有父级container指针
            moveit::task_constructor::ContainerBase* container;
            if (blackboard->get(container_bb_key, container)) {
                RCLCPP_INFO(node_->get_logger(), "Adding stage %s to parent container %s", 
                        stage->name().c_str(), container_bb_key.c_str());
                container->add(std::move(stage));
                return;
            }
            // 查找是否有父级wrapper
            std::string wrapper_bb_key = WRAPPER_PTR_PREFIX + parent_uid;
            moveit::task_constructor::WrapperBase* wrapper;
            if (blackboard->get(wrapper_bb_key, wrapper)) {
                RCLCPP_INFO(node_->get_logger(), "Adding stage %s to parent wrapper %s", 
                        stage->name().c_str(), wrapper_bb_key.c_str());
                wrapper->insert(std::move(stage));
                return;
            }
        }
        task_->add(std::move(stage));
    }

};

}