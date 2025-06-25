#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class GenerateGraspPoseBTNode : public MTCActionBaseNode {
public:
    GenerateGraspPoseBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for the this step"),
            BT::InputPort<std::string>(OBJECT_NAME, "name for the object"),
            BT::InputPort<std::string>(MONITORED_STAGE, "name for the monitored step"),
        };
    }

    BT::NodeStatus tick() override {
        std::string step_name, object_name, monitored_stage;
        getInput(STEP_NAME, step_name);
        getInput(OBJECT_NAME, object_name);
        getInput(MONITORED_STAGE, monitored_stage);

        // Sample grasp pose candidates in angle increments around the z-axis of the object
        auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>(step_name);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        stage->properties().set(MARK_NS, step_name);
        stage->setPreGraspPose(params_.hand_open_pose);
        stage->setObject(params_.object_name);  // object to sample grasps for
        stage->setAngleDelta(params_.angle_delta); // default： M_PI / 12

        // hook into successful initial-phase solutions
        moveit::task_constructor::Stage* monitored_state_ptr = task_->findChild(monitored_stage);
        stage->setMonitoredStage(monitored_state_ptr);  

        addChildToParent(std::move(stage));

        return BT::NodeStatus::SUCCESS;
    }
};

} // moveit_mtc_bt_node
