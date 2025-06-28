#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <memory>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/passthrough.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/decorator/mtc_decorator_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"
#include <moveit/task_constructor/cost_terms.h>

namespace moveit_mtc_bt_nodes {

class PassThroughDecorator : public MTCWrapperBaseNode {
public:
    PassThroughDecorator(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<rclcpp::Node> nh,
                         const std::shared_ptr<moveit::task_constructor::Task> task,
                         const moveit_mtc_bt_parameters::Params& params)
        : MTCWrapperBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "Name for the PassThrough wrapper"),
            BT::InputPort<std::string>(COST_TERM_TYPE, "", "Type of cost term to use (supported types: 'path_length', 'trajectory_duration', 'link_motion', 'constant')"),
            BT::InputPort<std::string>(COST_TERM_PARAM, "Parameters for the cost term (required for 'link_motion' (link name) and 'constant' (numeric value), unused for 'path_length' and 'trajectory_duration')")
        };
    }

    // 创建并配置PassThrough包装器
    virtual std::unique_ptr<moveit::task_constructor::WrapperBase> createWrapper() override {
        std::string step_name;
        getInput(STEP_NAME, step_name);
        
        // 创建PassThrough包装器
        auto wrapper = std::make_unique<moveit::task_constructor::stages::PassThrough>(step_name);
        
        // 从任务参数继承属性
        wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        
        // 设置CostTerm（如果提供了相关参数）
        std::string cost_term_type;
        std::string cost_term_param;
        if (getInput(COST_TERM_TYPE, cost_term_type) && !cost_term_type.empty()) {
            getInput(COST_TERM_PARAM, cost_term_param);
            
            // 根据类型创建相应的CostTerm
            moveit::task_constructor::CostTermConstPtr cost_term;
            if (cost_term_type == PATH_LENGTH) {
                cost_term = std::make_unique<moveit::task_constructor::cost::PathLength>();
            } else if (cost_term_type == TRAJECTORY_DURATION) {
                cost_term = std::make_unique<moveit::task_constructor::cost::TrajectoryDuration>();
            } else if (cost_term_type == LINK_MOTION) {
                cost_term = std::make_unique<moveit::task_constructor::cost::LinkMotion>(cost_term_param);
            } else if (cost_term_type == CONSTANT) {
                double value = 0.0;
                try {
                    value = std::stod(cost_term_param);
                } catch (...) {
                    // 如果转换失败，使用默认值0.0
                }
                cost_term = std::make_unique<moveit::task_constructor::cost::Constant>(value);
            }
            
            if (cost_term) {
                wrapper->setCostTerm(cost_term);
            }
        }
        
        return wrapper;
    }

    virtual std::string getWrapperType() override {
        return PASS_THROUGH;
    }
};

} // namespace moveit_mtc_bt_nodes