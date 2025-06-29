#!/bin/bash
set -e

# 将ROS 2发行版设置为变量
# 如果传入了参数，则使用传入的参数，否则使用默认值"jazzy"
if [ $# -gt 0 ]; then
    ROS_DISTRO="$1"
else
    ROS_DISTRO="jazzy"
fi

source /opt/ros/$ROS_DISTRO/setup.bash

# 导航回工作区根目录
cd /root/ros2_ws

# 为所有包安装ROS2依赖
echo "Installing ROS 2 dependencies..."
sudo rosdep fix-permissions 
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

# 现在在依赖安装完成后应用所有修复

# 修复storage.cpp
echo "Fixing storage.cpp..."
cd /root/ros2_ws/src/moveit_task_constructor
if [ -f core/src/storage.cpp ]; then
    # 创建备份
    cp core/src/storage.cpp core/src/storage.cpp.backup

    # 使用sed替换四行为新行
    sed -i '/if (this->end()->scene()->getParent() == this->start()->scene())/,+3c\    this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);' core/src/storage.cpp || echo "警告：无法修改storage.cpp"
fi

# 修复cartesian_path.cpp
echo "Fixing cartesian_path.cpp..."
if [ -f core/src/solvers/cartesian_path.cpp ]; then
    # 创建备份
    cp core/src/solvers/cartesian_path.cpp core/src/solvers/cartesian_path.cpp.backup

    # 进行替换
    sed -i 's/moveit::core::JumpThreshold(props.get<double>("jump_threshold")), is_valid,/moveit::core::JumpThreshold::relative(props.get<double>("jump_threshold")), is_valid,/' core/src/solvers/cartesian_path.cpp || echo "警告：无法修改cartesian_path.cpp"
fi

cd /root/ros2_ws

# 构建包
echo "Building packages..."

# 优先构建moveit_task_constructor
colcon build --packages-skip bt_moveit2_mtc_nodes bt_service_interfaces behavior_tree_executor --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 再进行构建完整项目
colcon build 
source install/setup.bash

echo "Workspace setup completed!"