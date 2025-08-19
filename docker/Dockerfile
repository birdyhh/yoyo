# 将ROS发行版设置为参数，默认为'jazzy'
ARG ROS_DISTRO=jazzy

# 基础镜像来自Docker Hub上的官方ROS仓库
# 您可以在这里找到可用的ROS镜像：https://hub.docker.com/_/ros/tags
# 我们使用ros-base镜像，它包含核心ROS 2包
FROM ros:${ROS_DISTRO}-ros-base

# 为这个Dockerfile设置维护者信息
LABEL maintainer="birdyhh"

# 设置环境变量
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# 更新系统并安装基本工具
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends apt-utils wget curl git build-essential \
    vim sudo lsb-release locales bash-completion tzdata gosu gedit htop nano libserial-dev

# 安装ROS 2开发所需的额外工具
RUN apt-get update -q && \
    apt-get install -y gnupg2 iputils-ping usbutils \
    python3-argcomplete python3-colcon-common-extensions python3-networkx python3-pip python3-rosdep python3-vcstool

# 设置ROS 2环境
RUN rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /root/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# 安装额外的ROS 2包
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-moveit

# 安装Mesa图形驱动程序
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    DEBIAN_FRONTEND=noninteractive add-apt-repository ppa:kisak/kisak-mesa

# 创建必要的目录
RUN mkdir -p /etc/udev/rules.d && \
    mkdir -p /root/ros2_ws/src/ 

# 将整个项目复制到容器中
# 由于我们使用父目录作为上下文，因此可以复制所有内容
COPY ./src /root/ros2_ws/src/

# 从docker目录复制配置文件
# 注意docker/前缀，因为我们在父上下文中
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/

# 使脚本可执行
RUN chmod +x /root/workspace.sh /root/entrypoint.sh

# 运行工作区设置脚本
WORKDIR /root
RUN ./workspace.sh ${ROS_DISTRO}

# 确保在每个新shell中都source ROS 2工作区
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# 设置容器的入口点
ENTRYPOINT ["/root/entrypoint.sh", "${ROS_DISTRO}"]

# 设置默认命令
CMD ["/bin/bash", "-c", "tail -f /dev/null"]