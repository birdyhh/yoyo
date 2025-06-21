#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "constants.hpp"

namespace moveit_mtc_bt_nodes {

class GenerateGraspPoseBTNode : public BT::SyncActionNode {
public:
    GenerateGraspPoseBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : BT::SyncActionNode(name, config), node_(nh), task_(task), params_(params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "move relative", "name for the this step")
        };
    }

    BT::NodeStatus tick() override {
        std::string step_name;
        getInput(STEP_NAME, step_name);
        moveit::task_constructor::Stage* initial_state_ptr = config().blackboard->get<moveit::task_constructor::Stage*>(CURRENT_STATE_PTR);

        // Sample grasp pose candidates in angle increments around the z-axis of the object
        auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>(step_name);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        stage->properties().set(MARK_NS, step_name);
        stage->setPreGraspPose(params_.hand_open_pose);
        stage->setObject(params_.object_name);  // object to sample grasps for
        stage->setAngleDelta(params_.angle_delta); // default： M_PI / 12
        stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

        // Compute IK for sampled grasp poses
        auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>(step_name + " IK", std::move(stage));
        wrapper->setMaxIKSolutions(params_.max_ik_solutions);  // limit number of solutions
        wrapper->setMinSolutionDistance(params_.min_solution_distance);
        // define virtual frame to reach the target_pose
        wrapper->setIKFrame(vectorToEigen(params_.grasp_frame_transform), params_.hand_frame);
        wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { EEF, GROUP });  // inherit properties from parent
        wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE,
                                                { TARGET_POSE });  // inherit property from child solution
        task_->insert(std::move(wrapper));

        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;


    Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
        return Eigen::Translation3d(values[0], values[1], values[2]) *
                Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
    }

};

} // moveit_mtc_bt_node
