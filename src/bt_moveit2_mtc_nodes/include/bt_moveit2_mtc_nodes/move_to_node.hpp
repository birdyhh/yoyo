#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "constants.hpp"

namespace moveit_mtc_bt_nodes {

class MoveToBTNode : public BT::SyncActionNode {
public:
    MoveToBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : BT::SyncActionNode(name, config), node_(nh), task_(task), params_(params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "move to", "name for the this step"),
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
        std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface> solver = createPlanner(planner_type);

        // 创建并配置 stage
        auto move_to = std::make_unique<moveit::task_constructor::stages::MoveTo>(step_name, solver);
        move_to->setGroup(group);
        move_to->setGoal(goal);

        // 添加到 task
        task_->add(std::move(move_to));

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
