from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    bridge1 = Node(
        package='ros2test',
        executable='qcar2_bridge',
        name='qcar2_bridge_' + str(1),
        parameters=[{"qcar_id":1}]
    )
    
    bridge2 = Node(
        package='ros2test',
        executable='qcar2_bridge',
        name='qcar2_bridge_' + str(2),
        parameters=[{"qcar_id":2}]
    )
    
    # finalize
    ld.add_action(bridge1)
    ld.add_action(bridge2)

    return ld
