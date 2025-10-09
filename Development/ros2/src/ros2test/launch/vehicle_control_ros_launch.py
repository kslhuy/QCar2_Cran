import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ros2test'),
        'config',
        'launch_config.yaml'
    )
    
    # Include launch file from qcar2_nodes package
    qcar2_virtual_launch_path = os.path.join(
        get_package_share_directory('qcar2_nodes'),
        'launch',
        'qcar2_launch.py'
    )

    include_qcar2_virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qcar2_virtual_launch_path)
    )
    
    ekf = Node(
        package='ros2test',
        executable='ekf',
        name='ekf',
        parameters=[config]
    )

    vehicle_control_ros = Node(
        package='ros2test',
        executable='vehicle_control_ros',
        name='vehicle_control_ros',
        parameters=[config]
    )

    lidar_occupancy_node = Node(
        package='ros2test',
        executable='lidar_occupancy_node',
        name='lidar_occupancy_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(include_qcar2_virtual_launch)
    ld.add_action(ekf)
    ld.add_action(vehicle_control_ros)
    ld.add_action(lidar_occupancy_node)

    return ld
