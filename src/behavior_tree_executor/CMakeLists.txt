cmake_minimum_required(VERSION 3.8)
project(behavior_tree_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# ######################
# add dependencies   #
# ######################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bt_service_interfaces REQUIRED)
find_package(bt_moveit2_mtc_nodes REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_components
    rcpputils
    behaviortree_cpp
    bt_service_interfaces
    bt_moveit2_mtc_nodes
    std_msgs
    Eigen3 )

# ###############
# add include   #
# ###############
include_directories(
  include
)

# ##########
# BUILD  #
# ##########
add_executable(behavior_tree_executor_node src/behavior_tree_executor_node.cpp src/behavior_tree_executor.cpp)
target_compile_features(behavior_tree_executor_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(behavior_tree_executor_node ${THIS_PACKAGE_DEPS})

# ############
# INSTALL  #
# ############
install(TARGETS
  behavior_tree_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

# ############
# TESTING  #
# ############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
