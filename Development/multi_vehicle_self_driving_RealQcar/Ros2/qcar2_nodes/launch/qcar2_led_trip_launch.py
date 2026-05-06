from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qcar2_nodes',
            executable='qcar2_ledTrip',
            name='qcar2_led_trip',
            parameters=[{
                'device_type': 'physical',
                'led_color_id': 0,
            }],
        ),
    ])
