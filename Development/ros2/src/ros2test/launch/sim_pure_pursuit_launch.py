from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ros2test'),
        'config',
        'sim_config.yaml'
    )
    pure_pursuit = Node(
        package='ros2test',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[config]
    )

    waypoints = Node(
        package='ros2test',
        executable='waypoints',
        name='waypoints',
        parameters=[config]
    )
    # finalize
    ld.add_action(pure_pursuit)
    ld.add_action(waypoints)

    return ld
