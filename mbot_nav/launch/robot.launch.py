#!/usr/bin/env python3
"""
One-shot robot startup:
  1. Starts foxglove_bridge (ws://<robot-ip>:8765)
  2. Resets odometry via the firmware's /reset_odometry service
  3. Starts controller_node once the odom reset has finished

Usage:
  ros2 launch mbot_nav robot.launch.py
  ros2 launch mbot_nav robot.launch.py use_localization:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_localization = LaunchConfiguration('use_localization')
    foxglove_port = LaunchConfiguration('foxglove_port')

    # 1. Foxglove bridge
    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            ])
        ),
        launch_arguments={'port': foxglove_port}.items(),
    )

    # 2. Reset odometry. Gives up after 10 s so the launch can't hang
    #    if the micro-ROS agent or firmware isn't up.
    reset_odom = ExecuteProcess(
        cmd=['timeout', '10',
             'ros2', 'service', 'call',
             '/reset_odometry', 'std_srvs/srv/Trigger', '{}'],
        name='reset_odometry',
        output='screen',
    )

    # 3. Controller, started only after the odom reset finishes
    controller = Node(
        package='mbot_nav',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{'use_localization': use_localization}],
    )

    start_controller_after_reset = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odom,
            on_exit=[
                LogInfo(msg='Odometry reset done, starting controller_node'),
                controller,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_localization', default_value='false',
            description='true: use TF map->base_footprint, false: use /odom'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='Foxglove bridge websocket port'),

        foxglove,
        reset_odom,
        start_controller_after_reset,
    ])
