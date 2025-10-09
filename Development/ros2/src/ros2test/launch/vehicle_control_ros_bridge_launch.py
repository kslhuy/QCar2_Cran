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
        'ekf_launch_config.yaml'
    )
    
    bridge1 = Node(
        package='ros2test',
        executable='qcar2_bridge',
        name='qcar2_bridge_' + str(1),
        parameters=[{"qcar_id":1}]
    )
    
    vehicle_control_ros_bridge1 = Node(
        package='ros2test',
        executable='vehicle_control_ros_bridge',
        name='vehicle_control_ros_bridge1',
        parameters=[config]
    )
    
    bridge2 = Node(
        package='ros2test',
        executable='qcar2_bridge',
        name='qcar2_bridge_' + str(2),
        parameters=[{"qcar_id":2}]
    )
    
    vehicle_control_ros_bridge2 = Node(
        package='ros2test',
        executable='vehicle_control_ros_bridge',
        name='vehicle_control_ros_bridge2',
        parameters=[config]
    )
    
    bridge3 = Node(
        package='ros2test',
        executable='qcar2_bridge',
        name='qcar2_bridge_' + str(3),
        parameters=[{"qcar_id":3}]
    )
    
    vehicle_control_ros_bridge3 = Node(
        package='ros2test',
        executable='vehicle_control_ros_bridge',
        name='vehicle_control_ros_bridge3',
        parameters=[config]
    )
    # finalize
    ld.add_action(bridge1)
    ld.add_action(vehicle_control_ros_bridge1)
    ld.add_action(bridge2)
    ld.add_action(vehicle_control_ros_bridge2)
    ld.add_action(bridge3)
    ld.add_action(vehicle_control_ros_bridge3)

    return ld
