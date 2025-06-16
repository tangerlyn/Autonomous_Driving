from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('lqr_pid'),
        'config',
        'sim_config.yaml'
    )

    lqr_pid_node = Node(
        package='lqr_pid',
        executable='lqr_pid',
        name='lqr_pid_node',
        parameters=[config]
    )

    waypoint_visualizer_node = Node(
        package='lqr_pid',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(lqr_pid_node)
    ld.add_action(waypoint_visualizer_node)

    return ld

