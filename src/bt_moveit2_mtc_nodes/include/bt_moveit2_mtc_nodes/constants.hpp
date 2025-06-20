#include <string>

namespace moveit_mtc_bt_nodes {

    // planner name
    static const std::string PLANNER_CARTESIAN = "Cartesian";
    static const std::string PLANNER_PIPELINE = "Pipeline";
    static const std::string PLANNER_JOINT_INTERPOLATION = "JointInterpolation";

    // stage properties name
    static const std::string GOAL_JOINT_TOLERANCER = "goal_joint_tolerance";


    // blackboard port name
    static const std::string STEP_NAME = "step_name";
    static const std::string GROUP = "group";
    static const std::string GOAL = "goal";
    static const std::string PLANNER_TYPE = "planner_type";
    static const std::string CURRENT_STATE_PTR = "current_state_ptr";

} // end moveit_mtc_bt_nodes