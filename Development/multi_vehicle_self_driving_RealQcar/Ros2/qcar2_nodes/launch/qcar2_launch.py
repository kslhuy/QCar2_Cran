# This is the launch file that starts up the basic QCar2 nodes

import subprocess

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, TimerAction, DeclareLaunchArgument)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import (OnProcessExit, OnProcessStart)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    speed_control_mode_arg = DeclareLaunchArgument(
        'speed_control_mode',
        default_value='throttle',
        description='Speed control mode: "velocity" (PD) or "throttle" (Raw PWM)'
    )
        
    lidar_node = Node(
            package='qcar2_nodes',
            executable='lidar',
            name='Lidar'
        )
    
    realsense_camera_node = Node(
            package='qcar2_nodes',
            executable='rgbd',
            name='RealsenseCamera'
        )
    
    csi_camera_node = Node(
            package='qcar2_nodes',
            executable='csi',
            name='DownwardFacingCamera'
        )
    
    qcar2_hardware = Node(
            package='qcar2_nodes',
            executable='qcar2_hardware',
            name='qcar2_hardware',
            parameters=[{
                'speed_control_mode': LaunchConfiguration('speed_control_mode')
            }]
        )
     
    return LaunchDescription([
        speed_control_mode_arg,
        lidar_node,
        # realsense_camera_node,
        # csi_camera_node,
        qcar2_hardware,
    ])
