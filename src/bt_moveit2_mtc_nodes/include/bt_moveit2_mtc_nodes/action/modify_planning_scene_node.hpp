#pragma once
#include <cmath>
#include <rclcpp/logging.hpp>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.hpp>
#include <vector>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class ModifyPlanningSceneBTNode : public MTCActionBaseNode {
public:
    ModifyPlanningSceneBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for the this step"),
            BT::InputPort<std::string>(MODIFY_SCENE, "scene to perform: remove_object, attach_enabled, collisions_enabled"),
            BT::InputPort<std::vector<std::string>>(OBJECT_NAMES, "", "An array of object names, separated by commas(,), for example: object1,object2"),
            BT::InputPort<std::string>(OTHER_OBJECT_TYPE, "", "robot(robot link) or object(object link)"),
            BT::InputPort<std::vector<std::string>>(OTHER_OBJECT_NAMES, "","An array of  other object names to control collision with, separated by commas (,), for example: link1, link2"),
            BT::InputPort<bool>(COLLISION_ENABLED, true, "allow or forbid collisions(attach) (true=allow, false=forbid)")
        };
    }

    BT::NodeStatus tick() override { 
        std::string step_name, modify_scene, other_object_type;
        getInput(STEP_NAME, step_name);
        getInput(MODIFY_SCENE, modify_scene);
        getInput(OTHER_OBJECT_TYPE, other_object_type);

        std::vector<std::string> object_names, other_object_names;
        getInput(OBJECT_NAMES, object_names);
        getInput(OTHER_OBJECT_NAMES, other_object_names);

        bool collision_enabled;
        getInput(COLLISION_ENABLED, collision_enabled);

        auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(step_name);

        if (modify_scene == REMOVE_OBJECT) {
            // 删除物体
            for (const auto& object_name : object_names) {
                RCLCPP_INFO(node_->get_logger(), "Removing object '%s'", object_name.c_str());
                stage->removeObject(object_name);
            }
        } else if (modify_scene == ATTACH_ENABLE) {
            for (const auto& attach_link : other_object_names) {
                RCLCPP_INFO(node_->get_logger(), "Attaching enable set %d between '%s' and objects", collision_enabled, attach_link.c_str());
                stage->attachObjects(object_names, attach_link, collision_enabled);
            }
        } else if (modify_scene == COLLISION_ENABLED) {
            if (other_object_type == ROBOT) {
                for (const auto& group_name : other_object_names) {
                    const auto& jmg = task_->getRobotModel()->getJointModelGroup(group_name);
                    RCLCPP_INFO(node_->get_logger(), "Collisions enable set %d between '%s' and objects", collision_enabled, group_name.c_str());
                    stage->allowCollisions(object_names, jmg->getLinkModelNamesWithCollisionGeometry(), collision_enabled);
                }              
            } else if (other_object_type == OBJECT) {
                for (const auto& link_name : other_object_names) {
                    RCLCPP_INFO_STREAM(node_->get_logger(), "Collisions enable between " << link_name << " and 'objects'.");
                }
                stage->allowCollisions(object_names, other_object_names, collision_enabled);
            }
        }  else {
            // 未知操作
            RCLCPP_ERROR(node_->get_logger(), "Unknown modify_scene: %s", modify_scene.c_str());
            return BT::NodeStatus::FAILURE;
        }
        
        addChildToParent(std::move(stage));
        return BT::NodeStatus::SUCCESS;
    }

};

} // moveit_mtc_bt_node
