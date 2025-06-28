#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <functional>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>

#include "bt_moveit2_mtc_nodes/decorator/mtc_decorator_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class PredicateFilterDecorator : public MTCWrapperBaseNode {
public:
    PredicateFilterDecorator(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<rclcpp::Node> nh,
                             const std::shared_ptr<moveit::task_constructor::Task> task,
                             const moveit_mtc_bt_parameters::Params& params)
        : MTCWrapperBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "Name for the PredicateFilter wrapper"),
            BT::InputPort<bool>(IGNORE_FILTER, false, "Ignore predicate and forward all solutions"),
            BT::InputPort<std::string>(PREDICATE_TYPE, "", "Type of predicate to use (e.g., 'always_true', 'always_false', 'cost_threshold')"),
            BT::InputPort<std::string>(PREDICATE_PARAM, "Parameters for the predicate (e.g., threshold value for cost_threshold)")
        };
    }

    // 创建并配置PredicateFilter包装器
    virtual std::unique_ptr<moveit::task_constructor::WrapperBase> createWrapper() override {
        std::string step_name;
        getInput(STEP_NAME, step_name);
        
        // 创建PredicateFilter包装器
        auto wrapper = std::make_unique<moveit::task_constructor::stages::PredicateFilter>(step_name);
        
        // 配置参数
        bool ignore_filter;
        if (getInput(IGNORE_FILTER, ignore_filter)) {
            wrapper->setIgnoreFilter(ignore_filter);
        }
        
        // 设置谓词函数
        std::string predicate_type;
        std::string predicate_param;
        std::function<bool(const moveit::task_constructor::SolutionBase& /*s*/, std::string& /*comment*/)> predicate_fn;
        if (getInput(PREDICATE_TYPE, predicate_type) && !predicate_type.empty()) {
            // 根据谓词类型设置不同的谓词函数
            if (predicate_type == ALWAYS_TRUE) {
                // 总是接受所有解决方案
                predicate_fn = [](const moveit::task_constructor::SolutionBase& /*s*/, std::string& /*comment*/) {
                    return true; 
                };
            } else if (predicate_type == ALWAYS_FALSE) {
                // 总是拒绝所有解决方案
                predicate_fn = [](const moveit::task_constructor::SolutionBase& /*s*/, std::string& /*comment*/) {
                    return false; 
                };
            } else if (predicate_type == COST_THRESHOLD) {
                // 基于成本阈值过滤解决方案
                double threshold = 10.0;
                if (getInput(PREDICATE_PARAM, predicate_param) && !predicate_param.empty()) {
                    try {
                        threshold = std::stod(predicate_param);
                    } catch (...) {
                        RCLCPP_WARN(node_->get_logger(), "Failed to parse predicate_param as double, using default value 0.0");
                    }
                }
                
                predicate_fn = [threshold](const moveit::task_constructor::SolutionBase& s, std::string& /*comment*/) {
                    return s.cost() <= threshold;
                };
            } else {
                RCLCPP_WARN(node_->get_logger(), "Unknown predicate_type: %s. Using default always_true predicate.", predicate_type.c_str());
                predicate_fn = [](const moveit::task_constructor::SolutionBase& /*s*/, std::string& /*comment*/) {
                    return true; 
                };
            }
        } else {
            // 如果没有指定谓词类型，使用默认的总是接受所有解决方案
            predicate_fn = [](const moveit::task_constructor::SolutionBase& /*s*/, std::string& /*comment*/) {
                return true; 
            };
        }
        wrapper->setPredicate(predicate_fn);
        
        // 从任务参数继承属性
        wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        
        return wrapper;
    }

    virtual std::string getWrapperType() override {
        return PREDICATE_FILTER;
    }
};

} // namespace moveit_mtc_bt_nodes