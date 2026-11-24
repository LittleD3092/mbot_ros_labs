#!/usr/bin/env python3
"""Checkpoint 3.2 exploration in simulation: fake MBot + the real stack.

Starts:
  1. sim/fake_mbot.py   (stands in for the robot + mbot_bringup)
  2. slam_node          (mbot_slam)
  3. navigation_node    (pose_source:=tf)
  4. controller_node    (use_localization:=true)
  5. exploration_node   (after explore_delay seconds, so SLAM has a first map)
  6. rviz2              (optional)

Usage (workspace root, after building mbot_nav and mbot_slam):
  ros2 launch sim/explore_sim.launch.py
  ros2 launch sim/explore_sim.launch.py map:=maze2
  ros2 launch sim/explore_sim.launch.py explore:=false   # set goals yourself in RViz
  ros2 launch sim/explore_sim.launch.py rviz:=false

Prefer `make sim-explore` from the host: it also cuts the container off from
the real robot (see the Makefile).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.dirname(SIM_DIR)


def fake_robot(context):
    m = LaunchConfiguration('map').perform(context)
    map_yaml = m if m.endswith('.yaml') else os.path.join(WS_DIR, 'mbot_nav', 'maps', f'{m}.yaml')
    args = ['python3', os.path.join(SIM_DIR, 'fake_mbot.py'), '--ros-args',
            '-p', f'map_yaml:={map_yaml}']
    for name in ('start_x', 'start_y', 'start_theta', 'seed'):
        args += ['-p', f'{name}:={LaunchConfiguration(name).perform(context)}']
    return [ExecuteProcess(cmd=args, name='fake_mbot', output='screen')]


def generate_launch_description():
    rviz_cfg = os.path.join(WS_DIR, 'mbot_nav', 'rviz', 'path_planning.rviz')

    stack = [
        Node(package='mbot_slam', executable='slam_node', name='slam_node', output='screen'),
        Node(package='mbot_nav', executable='navigation_node', name='navigation_node',
             output='screen', parameters=[{'pose_source': 'tf'}]),
        Node(package='mbot_nav', executable='controller_node', name='controller_node',
             output='screen', parameters=[{'use_localization': True}]),
    ]

    exploration = TimerAction(
        period=LaunchConfiguration('explore_delay'),
        actions=[Node(package='mbot_nav', executable='exploration_node',
                      name='exploration_node', output='screen')],
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                arguments=['-d', rviz_cfg], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='maze1',
                              description='Map name in mbot_nav/maps (maze1, maze2, final_map) '
                                          'or a path to a map YAML'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_theta', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='-1', description='-1 = random noise each run'),
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument('explore_delay', default_value='3.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=fake_robot),
        *stack,
        exploration,
        rviz,
    ])
