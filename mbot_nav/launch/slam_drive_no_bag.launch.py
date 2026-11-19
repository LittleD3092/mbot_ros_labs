#!/usr/bin/env python3
"""
Full SLAM run in one command (no rosbag recording; slam_drive.launch.py records one):
  1. foxglove_bridge              (ws://<robot-ip>:8765)
  2. mbot_bringup                 (robot description + lidar)
  3. reset odometry               (/reset_odometry service)
  4. slam_node + controller_node  (started once the odom reset finishes)
  5. cp_1_publisher               (sends the waypoints -> ROBOT STARTS DRIVING)

Usage:
  source ~/mbot_ros_labs/install/setup.bash
  ros2 launch mbot_nav slam_drive_no_bag.launch.py
  ros2 launch mbot_nav slam_drive_no_bag.launch.py drive:=false           # everything except the waypoints
  ros2 launch mbot_nav slam_drive_no_bag.launch.py path_delay:=10.0       # wait longer before driving

The controller always uses the robot's own odometry (/odom), not the SLAM pose.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive = LaunchConfiguration('drive')
    path_delay = LaunchConfiguration('path_delay')

    # 1. Foxglove bridge
    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])),
    )

    # 2. Bringup (robot_state_publisher, joint_state_publisher, lidar)
    bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mbot_bringup'), 'launch', 'mbot_bringup.launch.py'])),
    )

    # 3. Reset odometry (gives up after 10 s so the launch can't hang)
    reset_odom = ExecuteProcess(
        cmd=['timeout', '10', 'ros2', 'service', 'call',
             '/reset_odometry', 'std_srvs/srv/Trigger', '{}'],
        name='reset_odometry',
        output='screen',
    )

    # 4. SLAM + controller
    slam = Node(
        package='mbot_slam',
        executable='slam_node',
        name='mbot_slam_node',
        output='screen',
    )

    controller = Node(
        package='mbot_nav',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{'use_localization': False}],  # always follow the robot's own /odom
    )

    # 5. Waypoints, sent path_delay seconds after slam/controller start
    waypoints = TimerAction(
        period=path_delay,
        actions=[
            LogInfo(msg='Publishing cp_1 waypoints - robot will start driving'),
            Node(
                package='mbot_nav',
                executable='cp_1_publisher',
                name='cp_1_publisher',
                output='screen',
            ),
        ],
        condition=IfCondition(drive),
    )

    after_reset = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_odom,
            on_exit=[
                LogInfo(msg='Odometry reset done, starting slam_node and controller_node'),
                slam,
                controller,
                waypoints,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='true',
                              description='Publish cp_1 waypoints so the robot drives'),
        DeclareLaunchArgument('path_delay', default_value='5.0',
                              description='Seconds to wait after slam/controller start before driving'),
        foxglove,
        bringup,
        reset_odom,
        after_reset,
    ])
