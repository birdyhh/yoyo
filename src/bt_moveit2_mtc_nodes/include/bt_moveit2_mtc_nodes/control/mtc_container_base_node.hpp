#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/controls/sequence_node.h>
#include <behaviortree_cpp/tree_node.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/logging.hpp>
#include <string>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class MTCContainerBaseNode : public BT::ControlNode {
public:
    MTCContainerBaseNode(const std::string& name,  const BT::NodeConfiguration& config,
                       const std::shared_ptr<rclcpp::Node> nh,
                       const std::shared_ptr<moveit::task_constructor::Task> task,
                       const moveit_mtc_bt_parameters::Params& params)
        : BT::ControlNode(name, config), node_(nh), task_(task), params_(params) {
        setRegistrationID("MTCContainerBaseNode");
    }
    virtual ~MTCContainerBaseNode() = default;

    void init(const std::unordered_map<uint16_t, uint16_t>& uid_map) {
        parent_uid_map_ = uid_map;
    }

protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;
    std::unique_ptr<moveit::task_constructor::ContainerBase> container_;
    std::unordered_map<uint16_t, uint16_t> parent_uid_map_;

protected:
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

} // namespace moveit_mtc_bt_nodes