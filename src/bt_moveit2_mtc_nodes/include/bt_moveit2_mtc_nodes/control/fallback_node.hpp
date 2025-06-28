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

// Fallbacks container node - tries children in sequence until one succeeds
class MTCFallbacksNode : public moveit_mtc_bt_nodes::MTCParallelContainerNode {
public:
    MTCFallbacksNode(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<rclcpp::Node> nh,
                     const std::shared_ptr<moveit::task_constructor::Task> task,
                     const moveit_mtc_bt_parameters::Params& params)
        : MTCParallelContainerNode(name, config, nh, task, params), current_active_child_(0) {
        setRegistrationID("MTCFallbacks");
    }

    virtual ~MTCFallbacksNode() = default;

    virtual void halt() override {
        current_active_child_ = 0;
        MTCParallelContainerNode::halt();
    }

protected:
    size_t current_active_child_ = 0;

    virtual BT::NodeStatus tick() override {
        std::string container_name;
        getInput(CONTAINER_NAME, container_name);

        // Ensure container is created
        if (!container_) {
            container_ = std::make_unique<moveit::task_constructor::Fallbacks>(container_name);

            task_->properties().exposeTo(container_->properties(), { EEF, HAND, GROUP, IK_FRAME });
            container_->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                                       { EEF, HAND, GROUP, IK_FRAME });
        }

        // Store container reference in blackboard before executing children
        std::string blackboard_key = CONTAINER_PTR_PREFIX + std::to_string(BT::TreeNode::UID());
        config().blackboard->set(blackboard_key, (moveit::task_constructor::ContainerBase*)container_.get());
        RCLCPP_INFO(node_->get_logger(), "Adding container %s to blackboard as %s", name().c_str(),
                    blackboard_key.c_str());

        // Execute children in fallback fashion
        BT::NodeStatus status = tickFallbacks();

        // Add container to parent when all children are done
        if (status != BT::NodeStatus::RUNNING) {
            addChildToParent(std::move(container_));
        }

        return status;
    }

    BT::NodeStatus tickFallbacks() {
        const size_t children_count = children_nodes_.size();

        while (current_active_child_ < children_count) {
            TreeNode* current_child_node = children_nodes_[current_active_child_];
            const BT::NodeStatus child_status = current_child_node->executeTick();

            switch (child_status) {
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;

                case BT::NodeStatus::SUCCESS:
                    // Success, we're done
                    resetChildren();
                    current_active_child_ = 0;
                    return BT::NodeStatus::SUCCESS;

                case BT::NodeStatus::FAILURE:
                    // Move to next child
                    current_active_child_++;
                    break;

                case BT::NodeStatus::SKIPPED:
                    // Move to next child
                    current_active_child_++;
                    break;

                case BT::NodeStatus::IDLE:
                    throw BT::LogicError("[", name(), "]: A children should not return IDLE");
            }
        }

        // All children failed
        resetChildren();
        current_active_child_ = 0;
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus getParallelResult() override {
        // This won't be called for Fallbacks as we override the tick method
        return BT::NodeStatus::FAILURE;
    }
};
}

