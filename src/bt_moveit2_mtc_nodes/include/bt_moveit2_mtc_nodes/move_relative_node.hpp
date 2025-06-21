#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "constants.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace moveit_mtc_bt_nodes {

class MoveRelativeBTNode : public BT::SyncActionNode {
public:
    MoveRelativeBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : BT::SyncActionNode(name, config), node_(nh), task_(task), params_(params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "move relative", "name for the this step"),
            BT::InputPort<std::string>(PLANNER_TYPE, PLANNER_CARTESIAN, "use what kind of planner"),
            BT::InputPort<std::string>(LINK, "This link will move based on the world frame"),
            BT::InputPort<double>(X, 0.0, "displacement in the x-direction"),
            BT::InputPort<double>(Y, 0.0, "displacement in the y-direction"),
            BT::InputPort<double>(Z, 0.0, "displacement in the z-direction")
        };
    }

    BT::NodeStatus tick() override {
        std::string step_name, link, planner_type;
        getInput(STEP_NAME, step_name);
        getInput(LINK, link);
        getInput(PLANNER_TYPE, planner_type);

        double x, y, z;
        getInput(X, x);
        getInput(Y, y);
        getInput(Z, z);

        auto planner = createPlanner(planner_type);

        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(step_name, planner);
        stage->properties().set(MARK_NS, step_name);
        stage->properties().set(LINK, link);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(params_.move_relative_min_dist, params_.move_relative_max_dist);
        
        // Set downward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = params_.world_frame;
        vec.vector.x = x;
        vec.vector.y = y;
        vec.vector.z = z;
        stage->setDirection(vec);
        task_->insert(std::move(stage));

        return BT::NodeStatus::SUCCESS;
    }

    std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface> createPlanner(std::string planner_type) {
        if (planner_type == PLANNER_CARTESIAN) {
            auto solver = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
            solver->setMaxVelocityScalingFactor(params_.max_velocity_scaling_factor);
            solver->setMaxAccelerationScalingFactor(params_.max_acceleration_scaling_factor);
            solver->setStepSize(params_.step_size);
            return solver;
        } else if (planner_type == PLANNER_JOINT_INTERPOLATION) {
            auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
            interpolation_planner->setMaxVelocityScalingFactor(params_.max_velocity_scaling_factor);
            interpolation_planner->setMaxAccelerationScalingFactor(params_.max_acceleration_scaling_factor);
            return interpolation_planner;
        } else {
            auto solver = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
            solver->setProperty(GOAL_JOINT_TOLERANCER, params_.goal_joint_tolerance);
            return solver;
        }
    }

    private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;

};

} // moveit_mtc_bt_node
