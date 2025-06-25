#pragma once
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"
#include "bt_moveit2_mtc_nodes/utils.hpp"

namespace moveit_mtc_bt_nodes {

class ConnectBTNode : public MTCActionBaseNode {
public:
    ConnectBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for the this step"),
            BT::InputPort<std::vector<std::pair<std::string, std::string>>>(
                GROUP_PLANNER_MAP, "panda_arm:pipeline",
                "group-planner mapping (e.g. 'panda_arm:pipeline,hand:cartesian')")
        };
    }

    BT::NodeStatus tick() override { 
        std::string step_name;
        getInput(STEP_NAME, step_name);

        std::vector<std::pair<std::string, std::string>> group_planner_str_map;
        getInput(GROUP_PLANNER_MAP, group_planner_str_map);

        moveit::task_constructor::stages::Connect::GroupPlannerVector group_planner_vec;
        for (const auto& group_planner_str : group_planner_str_map) {
            auto planner = PlannerFactory::getInstance().getPlanner(group_planner_str.second);
            group_planner_vec.emplace_back(group_planner_str.first, planner);
        }

        auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(step_name, group_planner_vec);
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

        addChildToParent(std::move(stage));

        return BT::NodeStatus::SUCCESS;
    }

};

} // moveit_mtc_bt_node
