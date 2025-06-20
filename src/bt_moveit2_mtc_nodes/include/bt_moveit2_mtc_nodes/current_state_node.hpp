#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "constants.hpp"

namespace moveit_mtc_bt_nodes {

class CurrentStateNode : public BT::SyncActionNode {
public:
    CurrentStateNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : BT::SyncActionNode(name, config), node_(nh), task_(task), params_(params) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>(STEP_NAME, "move to", "name for the this step")};
    }

    BT::NodeStatus tick() override { 
        std::string step_name;
        getInput(STEP_NAME, step_name);

        auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(step_name);

        // Forward current_state on to grasp pose generator
        moveit::task_constructor::Stage* current_state_ptr = nullptr;
        current_state_ptr = stage_state_current.get();
        config().blackboard->set(CURRENT_STATE_PTR, current_state_ptr);
        
        task_->add(std::move(stage_state_current));

        return BT::NodeStatus::SUCCESS;
    }

    private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::task_constructor::Task> task_;
    moveit_mtc_bt_parameters::Params params_;
};

} // moveit_mtc_bt_node
