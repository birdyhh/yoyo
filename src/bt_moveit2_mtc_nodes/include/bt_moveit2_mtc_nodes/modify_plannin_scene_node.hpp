#pragma once
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.hpp>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "constants.hpp"

namespace moveit_mtc_bt_nodes {

class ModifyPlanningSceneBTNode : public BT::SyncActionNode {
public:
    ModifyPlanningSceneBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : BT::SyncActionNode(name, config), node_(nh), task_(task), params_(params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "modify step", "name for the this step"),
            BT::InputPort<std::string>(MODIFY_SCENE, "modify scene", 
                "Scene to perform: attach, detach, remove_object, allow_collisions, forbid_collisions"),
            BT::InputPort<std::string>(OBJECT_NAME, "object name", "name for the object"),
            BT::InputPort<std::string>(ROBOT_PART, "robot part",
                "name for the robot part, e.g. group_name(allow_collisions, forbid_collisions) or link_name(attach, detach, remove_object)")
        };
    }

    BT::NodeStatus tick() override { 
        std::string step_name, action, object_name, robot_part;
        getInput(STEP_NAME, step_name);
        getInput(MODIFY_SCENE, action);
        getInput(OBJECT_NAME, object_name);
        getInput(ROBOT_PART, robot_part);

        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(step_name);

        if (action == "attach") {
            // 将物体附着到手部link
            stage->attachObject(object_name, robot_part);
        } else if (action == "detach") {
            // 将物体从手部link中分离
            stage->detachObject(object_name, robot_part);
        } else if (action == "remove_object") {
            // 删除物体
            stage->removeObject(object_name);
        } else if (action == "allow_collisions") {
            // 允许手部group与物体之间的碰撞
            stage->allowCollisions(object_name, 
                                  *task_->getRobotModel()->getJointModelGroup(robot_part), true);
        } else if (action == "forbid_collisions") {
            // 禁止手部group与物体之间的碰撞
            stage->allowCollisions(object_name, 
                                  *task_->getRobotModel()->getJointModelGroup(robot_part), false);
        } else {
            // 未知操作
            RCLCPP_ERROR(node_->get_logger(), "Unknown action: %s", action.c_str());
            return BT::NodeStatus::FAILURE;
        }
        
        task_->add(std::move(stage));
        return BT::NodeStatus::SUCCESS;
    }

    private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;
};

} // moveit_mtc_bt_node
