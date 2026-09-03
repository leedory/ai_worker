#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

import fcntl
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


_MINI_LEADER_LOCK_HANDLE = None


def acquire_single_instance_lock(_context):
    """Keep a process-scoped lock so two launches cannot share the serial buses."""
    global _MINI_LEADER_LOCK_HANDLE

    lock_path = '/tmp/ffw_lg2_mini_leader_ai.lock'
    lock_handle = open(lock_path, 'a+', encoding='utf-8')
    try:
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or 'unknown'
        lock_handle.close()
        reason = f'Mini Leader is already running (launch pid={owner_pid}).'
        return [
            LogInfo(msg=reason),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(str(os.getpid()))
    lock_handle.flush()
    _MINI_LEADER_LOCK_HANDLE = lock_handle
    return [LogInfo(msg=f'Acquired Mini Leader single-instance lock: {lock_path}')]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_mini_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'start_zenoh_router',
            default_value='true',
            description='Start a private rmw_zenohd router on 127.0.0.1:7447.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    start_zenoh_router = LaunchConfiguration('start_zenoh_router')
    zenoh_router = ExecuteProcess(
        name='rmw_zenohd',
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        additional_env={
            # Isaac Sim/local teleop mode: keep this router private to Omen.
            # This prevents a second Cyclo instance on the robot PC from
            # publishing duplicate /data/recording/status messages.
            'ZENOH_CONFIG_OVERRIDE': (
                'listen/endpoints=["tcp/127.0.0.1:7447"];'
                'transport/shared_memory/enabled=true'
            ),
        },
        output='both',
        condition=IfCondition(start_zenoh_router),
    )

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_lg2_mini_leader',
            'ffw_lg2_mini_leader_ai_hardware_controller.yaml',
        ]
    )

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('ffw_description'),
                    'urdf',
                    'ffw_lg2_mini_leader',
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            # '--controller-ros-args',
            # '-r /leader/joint_trajectory_command_broadcaster_left/joint_trajectory:='
            # '/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory',
            # '--controller-ros-args',
            # '-r /leader/joint_trajectory_command_broadcaster_right/joint_trajectory:='
            # '/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory',
            'joint_state_broadcaster',
            'joint_trajectory_command_broadcaster',
            'trigger_position_controller',
            # 'leader_position_controller',
            'joystick_controller',
        ],
        parameters=[robot_description],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'frame_prefix': 'leader_'}],
    )

    follower_joint_state_relay_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='follower_joint_state_relay',
        name='follower_joint_state_relay',
        output='both',
    )

    # leader_feedback_node = Node(
    #     package='ffw_joint_trajectory_command_broadcaster',
    #     executable='leader_feedback',
    #     name='leader_feedback',
    #     output='both',
    # )

    gripper_trigger_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='gripper_trigger',
        name='gripper_trigger',
        output='both',
    )

    # Execute process to publish position command
    position_command_process = ExecuteProcess(
        name='trigger_position_command',
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '50',
            '-t', '50',
            '-p', '50',
            '/leader/trigger_position_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            'data: [0.0, 0.0]',
        ],
    )

    # Note: leader_position_controller commands are now continuously published by
    # joint_trajectory_command_broadcaster (mirrors follower poses every cycle).
    # No initial-kick process needed.

    delay_position_command_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[position_command_process],
        )
    )

    shutdown_on_router_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=zenoh_router,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Mini Leader Zenoh router exited; stopping partial launch.'
                    )
                )
            ],
        ),
        condition=IfCondition(start_zenoh_router),
    )

    shutdown_on_control_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Mini Leader ros2_control_node exited; stopping partial launch.'
                    )
                )
            ],
        )
    )

    # Wrap everything in a namespace 'leader'
    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            follower_joint_state_relay_node,
            delay_position_command_after_controllers,
            # leader_feedback_node,
            gripper_trigger_node,
        ]
    )

    # Start discovery first; the mini leader remains independent of Isaac startup order.
    delayed_leader = TimerAction(period=1.0, actions=[leader_with_namespace])
    # Give a duplicate launch's shutdown event time to run before it can spawn any process.
    delayed_runtime = TimerAction(period=0.25, actions=[zenoh_router, delayed_leader])

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=acquire_single_instance_lock),
            shutdown_on_router_exit,
            shutdown_on_control_exit,
            delayed_runtime,
        ]
    )
