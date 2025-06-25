#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_mtc_bt_nodes {

class GeneratePlacePoseBTNode : public MTCActionBaseNode {
public:
    GeneratePlacePoseBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for the this step"),
            BT::InputPort<std::string>(OBJECT_NAME, "name for the object"),
            BT::InputPort<std::string>(MONITORED_STAGE, "name for the monitored step")
        };
    }

    BT::NodeStatus tick() override {
        std::string step_name, object_name, monitored_stage;
        getInput(STEP_NAME, step_name);
        getInput(OBJECT_NAME, object_name);
        getInput(MONITORED_STAGE, monitored_stage);

        // Get pick stage pointer from blackboard
        moveit::task_constructor::Stage* monitored_stage_ptr = task_->findChild(monitored_stage);

        // Sample grasp pose candidates in angle increments around the z-axis of the object
        auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePlacePose>(step_name);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { IK_FRAME });
        stage->properties().set(MARK_NS, step_name);
        stage->setObject(object_name);

        // Set target pose
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = params_.object_reference_frame;
        p.pose = vectorToPose(params_.place_pose);
        p.pose.position.z += 0.5 * params_.object_dimensions[0] + params_.place_surface_offset;
        stage->setPose(p);
        stage->setMonitoredStage(monitored_stage_ptr);  // hook into successful pick solutions

        addChildToParent(std::move(stage));
        
        return BT::NodeStatus::SUCCESS;
    }

private:

    Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
        return Eigen::Translation3d(values[0], values[1], values[2]) *
                Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
    }

    geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
        return tf2::toMsg(vectorToEigen(values));
    }

};

} // moveit_mtc_bt_node
