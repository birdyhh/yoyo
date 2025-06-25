#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"
#include "bt_moveit2_mtc_nodes/utils.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace moveit_mtc_bt_nodes {

class MoveRelativeBTNode : public MTCActionBaseNode {
public:
    MoveRelativeBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for the this step"),
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

        auto planner = PlannerFactory::getInstance().getPlanner(planner_type);

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

        addChildToParent(std::move(stage));

        return BT::NodeStatus::SUCCESS;
    }

};

} // moveit_mtc_bt_node
