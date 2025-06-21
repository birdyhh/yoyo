#pragma once
#include <string>

namespace moveit_mtc_bt_nodes {

    // planner name
    static const std::string PLANNER_CARTESIAN = "Cartesian";
    static const std::string PLANNER_PIPELINE = "Pipeline";
    static const std::string PLANNER_JOINT_INTERPOLATION = "JointInterpolation";

    // stage properties name
    static const std::string GOAL_JOINT_TOLERANCER = "goal_joint_tolerance";
    static const std::string LINK = "link";
    static const std::string MARK_NS = "marker_ns";
    static const std::string EEF = "eef";
    static const std::string TARGET_POSE = "target_pose";


    // blackboard port name
    static const std::string STEP_NAME = "step_name";
    static const std::string GROUP = "group";
    static const std::string GOAL = "goal";
    static const std::string PLANNER_TYPE = "planner_type";
    static const std::string CURRENT_STATE_PTR = "current_state_ptr";
    static const std::string X = "x";
    static const std::string Y = "y";
    static const std::string Z = "z";
    static const std::string MODIFY_SCENE = "modify_scene";
    static const std::string OBJECT_NAME = "object_name";
    static const std::string ROBOT_PART = "robot_part";

} // end moveit_mtc_bt_nodes