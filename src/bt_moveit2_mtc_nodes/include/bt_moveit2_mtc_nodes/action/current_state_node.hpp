#pragma once
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class CurrentStateBTNode : public MTCActionBaseNode {
public:
    CurrentStateBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>(STEP_NAME, "name for the this step")};
    }

    BT::NodeStatus tick() override { 
        std::string step_name;
        getInput(STEP_NAME, step_name);
        auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(step_name);
        
        addChildToParent(std::move(stage_state_current));

        return BT::NodeStatus::SUCCESS;
    }

};

} // moveit_mtc_bt_node
