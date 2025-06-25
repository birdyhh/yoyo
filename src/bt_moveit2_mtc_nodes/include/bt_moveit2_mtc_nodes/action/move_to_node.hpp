#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"
#include "bt_moveit2_mtc_nodes/utils.hpp"

namespace moveit_mtc_bt_nodes {

class MoveToBTNode : public MTCActionBaseNode {
public:
    MoveToBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for the this step"),
            BT::InputPort<std::string>(GROUP, "which group to move"),
            BT::InputPort<std::string>(GOAL, "which action goal want to achieve"),
            BT::InputPort<std::string>(PLANNER_TYPE, PLANNER_PIPELINE, "use what kind of planner")
        };
    }

    BT::NodeStatus tick() override {
        std::string step_name, goal, group, planner_type;
        getInput(STEP_NAME, step_name);
        getInput(GROUP, group);
        getInput(GOAL, goal);
        getInput(PLANNER_TYPE, planner_type);

        // 创建 planner
        auto solver = PlannerFactory::getInstance().getPlanner(planner_type);

        // 创建并配置 stage
        auto move_to = std::make_unique<moveit::task_constructor::stages::MoveTo>(step_name, solver);
        move_to->setGroup(group);
        move_to->setGoal(goal);

        addChildToParent(std::move(move_to));

        return BT::NodeStatus::SUCCESS;
    }

};

} // moveit_mtc_bt_node
