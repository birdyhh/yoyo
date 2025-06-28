#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/logging.hpp>
#include <string>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/control/mtc_container_base_node.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

// Base class for MTC parallel container nodes
class MTCParallelContainerNode : public MTCContainerBaseNode {
public:
    MTCParallelContainerNode(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<rclcpp::Node> nh,
                             const std::shared_ptr<moveit::task_constructor::Task> task,
                             const moveit_mtc_bt_parameters::Params& params)
        : MTCContainerBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(CONTAINER_NAME, "Name of the MTC container"),
        };
    }

    virtual ~MTCParallelContainerNode() = default;

    virtual void halt() override {
        current_child_idx_ = 0;
        completed_children_count_ = 0;
        ControlNode::halt();
    }

protected:
    size_t current_child_idx_ = 0;
    size_t completed_children_count_ = 0;

    BT::NodeStatus tickChildren() {
        const size_t children_count = children_nodes_.size();
        
        if (!isStatusActive(status())) {
            completed_children_count_ = 0;
        }

        setStatus(BT::NodeStatus::RUNNING);

        // Tick all children that are not yet completed
        for (current_child_idx_ = 0; current_child_idx_ < children_count; ++current_child_idx_) {
            TreeNode* current_child_node = children_nodes_[current_child_idx_];
            
            // Skip already completed children
            if (current_child_node->status() == BT::NodeStatus::SUCCESS || 
                current_child_node->status() == BT::NodeStatus::FAILURE) {
                continue;
            }

            const BT::NodeStatus child_status = current_child_node->executeTick();

            switch (child_status) {
                case BT::NodeStatus::RUNNING:
                    // Child is still running, continue monitoring
                    break;
                    
                case BT::NodeStatus::SUCCESS:
                case BT::NodeStatus::FAILURE:
                    // Child completed (either successfully or not)
                    completed_children_count_++;
                    break;

                case BT::NodeStatus::SKIPPED:
                    // Child was skipped
                    completed_children_count_++;
                    break;

                case BT::NodeStatus::IDLE:
                    throw BT::LogicError("[", name(), "]: A children should not return IDLE");
            }
        }

        // Check if all children have completed
        if (completed_children_count_ == children_count) {
            // Reset for next execution
            resetChildren();
            current_child_idx_ = 0;
            completed_children_count_ = 0;

            // Determine final status based on children results
            return getParallelResult();
        }

        // Still running if not all children completed
        return BT::NodeStatus::RUNNING;
    }

    virtual BT::NodeStatus getParallelResult() = 0;
};

} // namespace moveit_mtc_bt_nodes