cmake_minimum_required(VERSION 3.8)
project(bt_moveit2_mtc_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(THIS_PACKAGE_DEPS
    rclcpp
    moveit_task_constructor_core
    moveit_ros_planning_interface
    moveit_visual_tools 
    behaviortree_cpp
    std_msgs)

include_directories(
  include)

# ###############################
# generate parameter listener   #
# ###############################
set(PARAMETERS_YAML "param/moveit_mtc_bt_parameters.yaml")
set(GENERATED_HEADER "${PROJECT_NAME}_parameters.hpp")
generate_parameter_library(
  ${PROJECT_NAME}_parameters
  ${PARAMETERS_YAML}
)

# ###################
# generate target   #
# ###################
add_library(${PROJECT_NAME} src/bt_moveit2_nodes.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_parameters)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

install(DIRECTORY include/  DESTINATION include/) 

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

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()


        
