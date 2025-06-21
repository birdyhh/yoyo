#pragma once
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "constants.hpp"

namespace moveit_mtc_bt_nodes {

class ConnectNode : public BT::SyncActionNode {
public:
    ConnectNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : BT::SyncActionNode(name, config), node_(nh), task_(task), params_(params) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>(STEP_NAME, "connect", "name for the this step")};
    }

    BT::NodeStatus tick() override { 
        std::string step_name;
        getInput(STEP_NAME, step_name);

        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
	    sampling_planner->setProperty(GOAL_JOINT_TOLERANCER, params_.goal_joint_tolerance);


        auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
		    step_name, moveit::task_constructor::stages::Connect::GroupPlannerVector{ { params_.arm_group_name, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
		task_->add(std::move(stage));

        return BT::NodeStatus::SUCCESS;
    }

    private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;
};

} // moveit_mtc_bt_node
