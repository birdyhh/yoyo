#!/bin/bash

# 设置ROS发行版
# 获取传递给entrypoint的ROS发行版参数
ROS_DISTRO="$1"
shift # 移除第一个参数，剩下的参数用于exec

# 创建GUI应用程序所需的临时目录 - 根据Linux桌面标准要求
# 设置XDG运行时目录
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# 为方便起见设置基础路径
# ROS_WS是ROS 2工作区的路径
ROS_WS="/root/ros2_ws"

# SHARED_ROS2是共享ROS 2文件的路径
SHARED_ROS2="/root/shared/ros2"

# 如果ros_domain_id.txt文件不存在，则创建该文件并设置默认值0
# 该文件存储ROS_DOMAIN_ID，用于多机器人通信
ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
    # 如果文件不存在，创建它并设置默认值0
    echo "0" > "$ROS_DOMAIN_ID_FILE"
    echo "Created $ROS_DOMAIN_ID_FILE with default value 0"
fi

# 需要通信的计算机必须使用相同的ROS_DOMAIN_ID。
# 该值可以是[0,232]之间的任何数字。如果您在相同环境中有多台机器人，这很有用。
# 如果.bashrc中尚未存在ROS_DOMAIN_ID，则更新它
if ! grep -q "export ROS_DOMAIN_ID" /root/.bashrc; then
  # 从文件中读取ROS_DOMAIN_ID
  ros_domain_id=$(cat "$ROS_DOMAIN_ID_FILE")
  # 如果.bashrc中还没有该导出命令，则添加它
  echo "export ROS_DOMAIN_ID=$ros_domain_id" >> /root/.bashrc
fi

# 使ROS Domain ID对当前脚本可用
# 这允许当前脚本使用ROS_DOMAIN_ID
export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

# 设置ROS控制台输出格式
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]  [{time}] [{name}]  {file_name}:{function_name}:{line_number} -> {message} '

# 源本地工作区设置
source $ROS_WS/install/setup.bash

# 源.bashrc文件以确保所有环境变量都是最新的
source /root/.bashrc

# 更改到ROS 2工作区目录
cd $ROS_WS

# 使用colcon构建ROS 2工作区
colcon build

# 更改回根目录
cd

# 重启一下ros2 daemon
ros2 daemon stop
ros2 daemon start

# 再次源.bashrc以确保应用构建过程中所做的任何更改
source /root/.bashrc

# 在此脚本运行后执行的任何命令都在脚本设置的环境中运行。
# 执行用户传递的命令（如果有的话）
if [ $# -gt 0 ]; then
    exec "$@"
else
    # 如果没有用户命令，则执行默认命令
    exec /bin/bash -c "tail -f /dev/null"
fi