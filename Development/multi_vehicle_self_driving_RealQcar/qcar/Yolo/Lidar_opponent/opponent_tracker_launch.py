#!/usr/bin/env python3
"""
Launch file for the Opponent Detection & Tracking system on QCar2.

Usage:
    ros2 launch <this_file>

Or run directly:
    python3 opponent_tracker_launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve config path relative to this file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(this_dir, 'opponent_tracker_params.yaml')

    detector_node = Node(
        executable=os.path.join(this_dir, 'opponent_detector_node.py'),
        name='opponent_detector_node',
        output='screen',
        parameters=[config_file] if os.path.exists(config_file) else [],
    )

    tracker_node = Node(
        executable=os.path.join(this_dir, 'opponent_tracker_node.py'),
        name='opponent_tracker_node',
        output='screen',
        parameters=[config_file] if os.path.exists(config_file) else [],
    )

    return LaunchDescription([
        detector_node,
        tracker_node,
    ])
