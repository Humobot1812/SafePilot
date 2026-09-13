#!/usr/bin/env python3
"""
Alias launch file for simulation.launch.py
Usage:
  ros2 launch safe_pilot sim.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sim_launch_file = os.path.join(current_dir, 'simulation.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file)
        )
    ])
