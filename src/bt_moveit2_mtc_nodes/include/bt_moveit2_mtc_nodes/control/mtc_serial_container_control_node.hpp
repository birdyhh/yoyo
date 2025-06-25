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

class MTCSerialContainerNode : public BT::ControlNode {
public:
    MTCSerialContainerNode(const std::string& name,  const BT::NodeConfiguration& config,
                       const std::shared_ptr<rclcpp::Node> nh,
                       const std::shared_ptr<moveit::task_constructor::Task> task,
                       const moveit_mtc_bt_parameters::Params& params)
        : BT::ControlNode(name, config), node_(nh), task_(task), params_(params), current_child_idx_(0) {
        setRegistrationID("MTCSerialContainer");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(CONTAINER_NAME, "Name of the MTC container"),
        };
    }

    virtual ~MTCSerialContainerNode() = default;

    virtual void halt() override
    {
        current_child_idx_ = 0;
        skipped_count_ = 0;
        ControlNode::halt();
    }

    void init(const std::unordered_map<uint16_t, uint16_t>& uid_map) {
        parent_uid_map_ = uid_map;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;
    std::unique_ptr<moveit::task_constructor::SerialContainer> container_;
    size_t current_child_idx_;
    size_t skipped_count_ = 0;
    bool asynch_ = false;
    std::unordered_map<uint16_t, uint16_t> parent_uid_map_;

protected:
    virtual BT::NodeStatus tick() override {
        std::string container_name;
        getInput(CONTAINER_NAME, container_name);

        // 确保容器已创建
        if (!container_) {
            container_ = std::make_unique<moveit::task_constructor::SerialContainer>(container_name);

            task_->properties().exposeTo(container_->properties(), { EEF, HAND, GROUP, IK_FRAME });
		    container_->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { EEF, HAND, GROUP, IK_FRAME });
        }

        // 在执行子节点前，将容器引用存储到黑板上，key的命名为前缀+UID
        std::string blackboard_key = CONTAINER_PTR_PREFIX + std::to_string(BT::TreeNode::UID());
        config().blackboard->set(blackboard_key, (moveit::task_constructor::ContainerBase*)container_.get());
        RCLCPP_INFO(node_->get_logger(), "Adding container %s to blackboard as %s", name().c_str(), blackboard_key.c_str());

        // 执行子节点
        BT::NodeStatus status = tickLikeSquence();

        // 子节点执行成功后，将容器添加到任务中
        if (status == BT::NodeStatus::SUCCESS) {
            addChildToParent(std::move(container_));
        }

        return status;
    }

    BT::NodeStatus tickLikeSquence() {
        const size_t children_count = children_nodes_.size();

        if(!isStatusActive(status())) {
            skipped_count_ = 0;
        }

        setStatus(BT::NodeStatus::RUNNING);

        while(current_child_idx_ < children_count)
        {
            TreeNode* current_child_node = children_nodes_[current_child_idx_];

            auto prev_status = current_child_node->status();
            const BT::NodeStatus child_status = current_child_node->executeTick();

            switch(child_status)
            {
            case BT::NodeStatus::RUNNING: {
                return BT::NodeStatus::RUNNING;
            }
            case BT::NodeStatus::FAILURE: {
                // Reset on failure
                resetChildren();
                current_child_idx_ = 0;
                return child_status;
            }
            case BT::NodeStatus::SUCCESS: {
                current_child_idx_++;
                // Return the execution flow if the child is async,
                // to make this interruptible.
                if(asynch_ && requiresWakeUp() && prev_status == BT::NodeStatus::IDLE &&
                current_child_idx_ < children_count)
                {
                emitWakeUpSignal();
                return BT::NodeStatus::RUNNING;
                }
            }
            break;

            case BT::NodeStatus::SKIPPED: {
                // It was requested to skip this node
                current_child_idx_++;
                skipped_count_++;
            }
            break;

            case BT::NodeStatus::IDLE: {
                throw BT::LogicError("[", name(), "]: A children should not return IDLE");
            }
            }  // end switch
        }    // end while loop

        // The entire while loop completed. This means that all the children returned SUCCESS.
        const bool all_children_skipped = (skipped_count_ == children_count);
        if(current_child_idx_ == children_count)
        {
            resetChildren();
            current_child_idx_ = 0;
            skipped_count_ = 0;
        }
        // Skip if ALL the nodes have been skipped
        return (all_children_skipped) ? BT::NodeStatus::SKIPPED : BT::NodeStatus::SUCCESS;
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

} // namespace moveit_mtc_bt_nodes