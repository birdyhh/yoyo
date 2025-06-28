#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/logging.hpp>
#include <string>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"
#include "bt_moveit2_mtc_nodes/control/mtc_parallel_container_control_node.hpp"

namespace moveit_mtc_bt_nodes {
// alternatives container node - collects solutions from all children
class MTCAlternativesNode : public MTCParallelContainerNode {
public:
    MTCAlternativesNode(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<rclcpp::Node> nh,
                        const std::shared_ptr<moveit::task_constructor::Task> task,
                        const moveit_mtc_bt_parameters::Params& params)
        : MTCParallelContainerNode(name, config, nh, task, params) {
        setRegistrationID("MTCAlternatives");
    }

    virtual ~MTCAlternativesNode() = default;

protected:
    virtual BT::NodeStatus tick() override {
        std::string container_name;
        getInput(CONTAINER_NAME, container_name);

        // Ensure container is created
        if (!container_) {
            container_ = std::make_unique<moveit::task_constructor::Alternatives>(container_name);

            task_->properties().exposeTo(container_->properties(), { EEF, HAND, GROUP, IK_FRAME });
            container_->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                                       { EEF, HAND, GROUP, IK_FRAME });
        }

        // Store container reference in blackboard before executing children
        std::string blackboard_key = CONTAINER_PTR_PREFIX + std::to_string(BT::TreeNode::UID());
        config().blackboard->set(blackboard_key, (moveit::task_constructor::ContainerBase*)container_.get());
        RCLCPP_INFO(node_->get_logger(), "Adding container %s to blackboard as %s", name().c_str(),
                    blackboard_key.c_str());

        // Execute children
        BT::NodeStatus status = tickChildren();

        // Add container to parent when all children are done
        if (status != BT::NodeStatus::RUNNING) {
            addChildToParent(std::move(container_));
        }

        return status;
    }

    BT::NodeStatus getParallelResult() override {
        // For Alternatives, we succeed if any child succeeded
        // But since this is a parallel execution, we succeed if all children complete
        // and at least one succeeded
        bool any_success = false;
        for (const auto& child : children_nodes_) {
            if (child->status() == BT::NodeStatus::SUCCESS) {
                any_success = true;
                break;
            }
        }
        
        return any_success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}