#!/usr/bin/env python3
"""
Full Drone Simulation Launch File
=================================
Automates the full simulation workflow in one command:
1. Starts Gazebo Harmonic simulation with the Iris Runway world:
   `gz sim -v4 -r iris_runway.sdf`
   (in ~/gz_ws/src/ardupilot_gazebo/worlds)

2. Starts ArduPilot SITL after a configurable delay (default 3s):
   `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console`
   (in ~/Documents/Documents/Drone/apm/ardupilot/ArduCopter)

3. Starts the ROS 2 Drone Teleop system after a configurable delay (default 15s):
   `ros2 launch drone_controller teleop.launch.py`
"""

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    home_dir = os.path.expanduser('~')

    # Working directories
    gz_world_dir = os.path.join(home_dir, 'gz_ws', 'src', 'ardupilot_gazebo', 'worlds')
    ardupilot_dir = os.path.join(
        home_dir, 'Documents', 'Documents', 'Drone', 'apm', 'ardupilot', 'ArduCopter'
    )
    autotest_dir = os.path.join(
        home_dir, 'Documents', 'Documents', 'Drone', 'apm', 'ardupilot', 'Tools', 'autotest'
    )

    # Gazebo plugin and model paths
    gz_plugin_dir = os.path.join(home_dir, 'gz_ws', 'src', 'ardupilot_gazebo', 'build')
    gz_models_dir = os.path.join(home_dir, 'gz_ws', 'src', 'ardupilot_gazebo', 'models')
    gz_worlds_dir = gz_world_dir

    # Environment setup to ensure plugins and tools are resolved
    env = dict(os.environ)

    existing_plugin_path = env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_plugin_path = (
        f"{gz_plugin_dir}:{existing_plugin_path}" if existing_plugin_path else gz_plugin_dir
    )

    existing_resource_path = env.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = (
        f"{gz_models_dir}:{gz_worlds_dir}:{existing_resource_path}"
        if existing_resource_path
        else f"{gz_models_dir}:{gz_worlds_dir}"
    )

    existing_path = env.get('PATH', '')
    path_with_autotest = (
        f"{autotest_dir}:{existing_path}" if autotest_dir not in existing_path else existing_path
    )

    proc_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'PATH': path_with_autotest,
    }

    # Launch Arguments
    ardupilot_delay_arg = DeclareLaunchArgument(
        'ardupilot_delay',
        default_value='3.0',
        description='Delay in seconds before starting ArduCopter SITL after Gazebo boots'
    )

    teleop_delay_arg = DeclareLaunchArgument(
        'teleop_delay',
        default_value='15.0',
        description='Delay in seconds before starting ROS 2 drone teleop after Gazebo boots'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='aerothon_ground2.sdf',
        description='Gazebo world file name inside ardupilot_gazebo/worlds'
    )

    # ── 1. Gazebo Simulation Process ──────────────────────────────────────────
    # Equivalent to:
    # cd ~/gz_ws/src/ardupilot_gazebo/worlds && gz sim -v4 -r iris_runway.sdf
    gz_sim_proc = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-r', LaunchConfiguration('world')],
        cwd=gz_world_dir,
        output='screen',
        additional_env=proc_env,
    )

    # ── 2. ArduCopter SITL Process ────────────────────────────────────────────
    # Equivalent to:
    # cd ~/Documents/Documents/Drone/apm/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
    ardupilot_proc = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduCopter',
            '-f', 'gazebo-iris',
            '--model', 'JSON',
            '--console'
        ],
        cwd=ardupilot_dir,
        output='screen',
        additional_env=proc_env,
    )

    delayed_ardupilot = TimerAction(
        period=LaunchConfiguration('ardupilot_delay'),
        actions=[
            LogInfo(msg='[Launch] ⏱ 3s delay reached. Starting ArduCopter SITL (sim_vehicle.py)...'),
            ardupilot_proc
        ]
    )

    # ── 3. ROS 2 Teleop System Launch ─────────────────────────────────────────
    # Equivalent to:
    # ros2 launch drone_controller teleop.launch.py
    current_launch_dir = os.path.dirname(os.path.abspath(__file__))
    teleop_launch_path = os.path.join(current_launch_dir, 'teleop.launch.py')

    if not os.path.exists(teleop_launch_path):
        try:
            pkg_share = get_package_share_directory('safe_pilot')
            teleop_launch_path = os.path.join(pkg_share, 'launch', 'teleop.launch.py')
        except (PackageNotFoundError, Exception):
            pass

    delayed_teleop = TimerAction(
        period=LaunchConfiguration('teleop_delay'),
        actions=[
            LogInfo(msg='[Launch] ⏱ Delay reached. Launching ROS 2 joystick & drone teleop nodes...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch_path)
            )
        ]
    )

    return LaunchDescription([
        ardupilot_delay_arg,
        teleop_delay_arg,
        world_arg,
        LogInfo(msg='[Launch] Starting Gazebo Harmonic simulation...'),
        gz_sim_proc,
        delayed_ardupilot,
        delayed_teleop,
    ])
