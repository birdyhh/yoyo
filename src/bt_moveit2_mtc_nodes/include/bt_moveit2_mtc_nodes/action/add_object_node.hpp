#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "bt_moveit2_mtc_nodes/action/mtc_action_base_node.hpp"
#include "bt_moveit2_mtc_nodes/bt_moveit2_mtc_nodes_parameters.hpp"
#include "bt_moveit2_mtc_nodes/constants.hpp"

namespace moveit_mtc_bt_nodes {

class AddObjectBTNode : public MTCActionBaseNode {
public:
    AddObjectBTNode(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<rclcpp::Node> nh,
                const std::shared_ptr<moveit::task_constructor::Task> task,
                const moveit_mtc_bt_parameters::Params& params)
        : MTCActionBaseNode(name, config, nh, task, params) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>(STEP_NAME, "name for this step"),
            BT::InputPort<std::string>(OBJECT_NAME, "name of the object to add"),
            BT::InputPort<std::string>(FRAME_ID, "world", "frame_id for the object"),
            BT::InputPort<double>(POSITION, "0.0,0.0,0.0,0.0,0.0,0.0", "x,y,z,roll,pitch,yaw position of the object"),
            BT::InputPort<uint8_t>(OBJECT_SHAPE, shape_msgs::msg::SolidPrimitive::BOX, "shape type of the object"),
            BT::InputPort<std::vector<double>>(OBJECT_DIMENSIONS, "dimensions of the object (BOX: [x,y,z], SPHERE: [radius], CYLINDER: [height,radius], CONE: [height,radius])")
        };
    }

    BT::NodeStatus tick() override { 
        std::string step_name, object_name, frame_id;
        getInput(STEP_NAME, step_name);
        getInput(OBJECT_NAME, object_name);
        getInput(FRAME_ID, frame_id);

        std::vector<double> position;
        getInput(POSITION, position);

        uint8_t shape_type;
        getInput(OBJECT_SHAPE, shape_type);

        std::vector<double> object_dimensions;
        getInput(OBJECT_DIMENSIONS, object_dimensions);

        // Create the collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = object_name;
        auto pose = vectorToPose(position);

        // Define the object shape
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_type;
        primitive.dimensions.resize(object_dimensions.size());
        for (size_t i = 0; i < object_dimensions.size(); ++i) {
            primitive.dimensions[i] = object_dimensions[i];
        }
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(collision_object);

        // // Create and configure the stage
        // auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(step_name);
        // stage->addObject(collision_object);

        // addChildToParentContainer(std::move(stage));
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
                // Set the object pose
        if (values.size() == 6) {
            return tf2::toMsg(vectorToEigen(values));
        } else {
            return tf2::toMsg(vectorToEigen(std::vector<double>(6, 0.0)));
        }
        
    }
};

} // moveit_mtc_bt_node