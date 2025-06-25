#pragma once
#include <string>

namespace moveit_mtc_bt_nodes {
    // planner name
    static const std::string PLANNER_CARTESIAN = "Cartesian";
    static const std::string PLANNER_PIPELINE = "Pipeline";
    static const std::string PLANNER_JOINT_INTERPOLATION = "JointInterpolation";
    // planner name lowercase
    static const std::string PLANNER_CARTESIAN_LC = "cartesian";
    static const std::string PLANNER_PIPELINE_LC = "pipeline";
    static const std::string PLANNER_JOINT_INTERPOLATION_LC = "jointinterpolation";

    // wrapper name
    static const std::string COMPUTE_IK = "ComputeIK";
    static const std::string PROPERTY_TYPE = "type";
    static const std::string PROPERTY_CONFIG = "config";

    // stage properties name
    static const std::string GOAL_JOINT_TOLERANCER = "goal_joint_tolerance";
    static const std::string LINK = "link";
    static const std::string MARK_NS = "marker_ns";
    static const std::string EEF = "eef";
    static const std::string TARGET_POSE = "target_pose";
    static const std::string IK_FRAME = "ik_frame";
    static const std::string HAND = "hand";

    // blackboard name
    static const std::string CONTAINER_PTR_PREFIX = "container_ptr_";
    static const std::string WRAPPER_PTR_PREFIX = "wrapper_ptr_";

    // port name
    static const std::string CONTAINER_NAME = "container_name";
    static const std::string STEP_NAME = "step_name";
    static const std::string GROUP = "group";
    static const std::string GOAL = "goal";
    static const std::string PLANNER_TYPE = "planner_type";
    static const std::string CURRENT_STATE_PTR = "current_state_ptr";
    static const std::string PICK_STATE_PTR = "pick_state_ptr";
    static const std::string X = "x";
    static const std::string Y = "y";
    static const std::string Z = "z";
    static const std::string DIRECTION = "direction";
    static const std::string WORLD = "world";
    static const std::string POSITION = "position";
    static const std::string MODIFY_SCENE = "modify_scene";
    static const std::string OBJECT_NAME = "object_name";
    static const std::string OBJECT_NAMES = "object_names";
    static const std::string ROBOT_GROUP = "robot_group";
    static const std::string MONITORED_STAGE = "monitored_step_name";
    static const std::string COLLISION_ENABLED = "collisions_enabled";
    static const std::string OTHER_OBJECT_TYPE = "other_object_type";
    static const std::string OTHER_OBJECT_NAMES = "other_object_names";
    static const std::string GROUP_PLANNER_MAP = "group_planner_map";
    static const std::string FRAME_ID = "frame_id";
    static const std::string OBJECT_SHAPE = "object_shape";
    static const std::string OBJECT_DIMENSIONS = "object_dimensions";
    static const std::string IK_FRAME_LINK = "ik_frame_link";
    static const std::string IK_FRAME_POSE = "ik_frame_pose";

    // ModifyPlanningScene properties
    static const std::string REMOVE_OBJECT = "remove_object";
    static const std::string ATTACH_ENABLE = "attach_enabled";
    static const std::string COLLISIONS_ENABLE = "collisions_enable";
    static const std::string ROBOT = "robot";
    static const std::string OBJECT = "object";

} // end moveit_mtc_bt_nodes