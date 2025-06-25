#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "behaviortree_cpp/basic_types.h"

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"


namespace moveit_mtc_bt_nodes { 

class PlannerFactory {
public:

    static PlannerFactory& getInstance();
    
    std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface> getPlanner(const std::string& planner_type);
    
    void init(const rclcpp::Node::SharedPtr& node, const moveit_mtc_bt_parameters::Params& params);

    virtual ~PlannerFactory() = default;

private:
    PlannerFactory() = default;
    
    PlannerFactory(const PlannerFactory&) = delete;
    PlannerFactory& operator=(const PlannerFactory&) = delete;
    
    std::unordered_map<std::string, std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface>> planners_map;
};

inline void PlannerFactory::init(const rclcpp::Node::SharedPtr& node, const moveit_mtc_bt_parameters::Params& params) {
    auto solver = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    solver->setMaxVelocityScalingFactor(params.max_velocity_scaling_factor);
    solver->setMaxAccelerationScalingFactor(params.max_acceleration_scaling_factor);
    solver->setStepSize(params.step_size);
    planners_map.insert(std::make_pair(PLANNER_CARTESIAN, solver));
    planners_map.insert(std::make_pair(PLANNER_CARTESIAN_LC, solver));

    auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
    interpolation_planner->setMaxVelocityScalingFactor(params.max_velocity_scaling_factor);
    interpolation_planner->setMaxAccelerationScalingFactor(params.max_acceleration_scaling_factor);
    planners_map.insert(std::make_pair(PLANNER_JOINT_INTERPOLATION, interpolation_planner));
    planners_map.insert(std::make_pair(PLANNER_JOINT_INTERPOLATION_LC, interpolation_planner));

    auto pipeline_solver = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);
    pipeline_solver->setProperty(GOAL_JOINT_TOLERANCER, params.goal_joint_tolerance);
    planners_map.insert(std::make_pair(PLANNER_PIPELINE, pipeline_solver));
    planners_map.insert(std::make_pair(PLANNER_PIPELINE_LC, pipeline_solver));
}

inline PlannerFactory& PlannerFactory::getInstance() {
    static PlannerFactory instance;
    return instance;
}

inline std::shared_ptr<moveit::task_constructor::solvers::PlannerInterface> PlannerFactory::getPlanner(const std::string& planner_type) {
    auto planner = planners_map.find(planner_type);
    if (planner != planners_map.end()) {
        return planner->second;
    }
    // 默认使用Pipeline
    return planners_map[PLANNER_PIPELINE];
}

} // namespace moveit_mtc_bt_nodes



namespace BT {

    template <>
    inline std::vector<std::string> convertFromString<std::vector<std::string>>(StringView str) {
        auto parts = splitString(str, ',');
        std::vector<std::string> result;
        result.reserve(parts.size());
        for(const auto& part : parts) {
            // 去除每个部分的前后空格
            std::string trimmed = std::string(part);
            trimmed.erase(0, trimmed.find_first_not_of(' '));
            trimmed.erase(trimmed.find_last_not_of(' ') + 1);
            result.push_back(trimmed);
        }
        return result;
    }

    template <>
    inline std::vector<std::pair<std::string, std::string>> convertFromString<std::vector<std::pair<std::string, std::string>>>(StringView str) {
        std::vector<std::pair<std::string, std::string>> planners;
        
        // 解析格式: "group1:planner1,group2:planner2,group3:planner3"
        auto pairs = splitString(str, ',');
        
        for (const auto& pair_str : pairs) {
            // 去除空格
            std::string trimmed_pair = std::string(pair_str);
            trimmed_pair.erase(0, trimmed_pair.find_first_not_of(' '));
            trimmed_pair.erase(trimmed_pair.find_last_not_of(' ') + 1);
            
            // 分割group和planner
            size_t colon_pos = trimmed_pair.find(':');
            if (colon_pos != std::string::npos) {
                std::string group = trimmed_pair.substr(0, colon_pos);
                std::string planner_type = trimmed_pair.substr(colon_pos + 1);
                
                // 去除可能的空格
                group.erase(0, group.find_first_not_of(' '));
                group.erase(group.find_last_not_of(' ') + 1);
                planner_type.erase(0, planner_type.find_first_not_of(' '));
                planner_type.erase(planner_type.find_last_not_of(' ') + 1);
                
                planners.push_back({group, planner_type});
            }
        }
        
        return planners;
    }

    template <>
    inline std::vector<double> convertFromString<std::vector<double>>(StringView str) {
        auto parts = splitString(str, ',');
        std::vector<double> result;
        result.reserve(parts.size());
        for(const auto& part : parts) {
            // 去除每个部分的前后空格
            std::string trimmed = std::string(part);
            trimmed.erase(0, trimmed.find_first_not_of(' '));
            trimmed.erase(trimmed.find_last_not_of(' ') + 1);
            
            double value = std::stod(trimmed);
            result.push_back(value);
        }
        return result;
    }
}
