from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

from generate_parameter_library_py.setup_helper import generate_parameter_module

def generate_launch_description():
    ld = LaunchDescription()

    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()


    pkg = "behavior_tree_executor"
    package_shared_path = get_package_share_directory(pkg)

    node = Node(
        package=pkg,
        namespace='',
        executable='behavior_tree_executor_node',
        name='behavior_tree_executor_node',
        output="screen",
        #arguments=['--ros-args', '--log-level', 'debug'], 
        parameters=[
            moveit_config,
            os.path.join(package_shared_path, 'param', 'parameters.yaml')
        ]
    )
    ld.add_action(node)

    return ld
