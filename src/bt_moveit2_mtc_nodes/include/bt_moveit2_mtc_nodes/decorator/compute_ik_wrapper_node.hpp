#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/decorator/mtc_decorator_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_mtc_bt_nodes {

class ComputeIKDecorator : public MTCWrapperBaseNode {
public:
    ComputeIKDecorator(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<rclcpp::Node> nh,
                       const std::shared_ptr<moveit::task_constructor::Task> task,
                       const moveit_mtc_bt_parameters::Params& params)
        : MTCWrapperBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "Name for the ComputeIK wrapper"),
            BT::InputPort<std::string>(IK_FRAME_LINK, "Link for IK frame"),
            BT::InputPort<std::vector<double>>(IK_FRAME_POSE, "Pose for IK frame (x,y,z,rx,ry,rz)")
        };
    }

    // 创建并配置ComputeIK包装器
    virtual std::unique_ptr<moveit::task_constructor::WrapperBase> createWrapper() override {
        std::string step_name;
        getInput(STEP_NAME, step_name);
        
        // 创建ComputeIK包装器
        auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>(step_name);
        
        // 配置默认参数
        wrapper->setMaxIKSolutions(params_.max_ik_solutions);
        wrapper->setTimeout(params_.ik_timeout);
        wrapper->setIgnoreCollisions(params_.ignore_collisions);
        wrapper->setMinSolutionDistance(params_.min_solution_distance);
        
        std::string ik_frame_link;
        std::vector<double> ik_frame_pose;
        if (getInput(IK_FRAME_LINK, ik_frame_link) && getInput(IK_FRAME_POSE, ik_frame_pose)) {
            if (ik_frame_pose.size() == 6) {
                Eigen::Isometry3d pose = Eigen::Translation3d(ik_frame_pose[0], ik_frame_pose[1], ik_frame_pose[2]) *
                                         Eigen::AngleAxisd(ik_frame_pose[3], Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(ik_frame_pose[4], Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(ik_frame_pose[5], Eigen::Vector3d::UnitZ());
                wrapper->setIKFrame(pose, ik_frame_link);
            } else {
                wrapper->setIKFrame(Eigen::Isometry3d::Identity(), ik_frame_link);
            }
        }
        
        // 从任务参数继承属性
        wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { EEF, GROUP });
        wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { TARGET_POSE });
        
        return wrapper;
    }

    virtual std::string getWrapperType() override {
        return COMPUTE_IK;
    }

    Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
        return Eigen::Translation3d(values[0], values[1], values[2]) *
            Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
    }

};

} // namespace moveit_mtc_bt_nodes