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
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


_A3_LOCK_HANDLE = None


def acquire_single_instance_lock(_context):
    """Prevent A3 and Mini launch processes from sharing the leader namespace/router."""
    global _A3_LOCK_HANDLE

    lock_path = '/tmp/ffw_lg2_mini_leader_ai.lock'
    lock_handle = open(lock_path, 'a+', encoding='utf-8')
    try:
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or 'unknown'
        lock_handle.close()
        reason = f'A leader launch is already running (launch pid={owner_pid}).'
        return [
            LogInfo(msg=reason),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(str(os.getpid()))
    lock_handle.flush()
    _A3_LOCK_HANDLE = lock_handle
    return [LogInfo(msg=f'Acquired A3 leader single-instance lock: {lock_path}')]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_a3.urdf.xacro',
            description='URDF/XACRO file for the A3 leader.',
        ),
        DeclareLaunchArgument(
            'controller_config_file',
            default_value='ffw_a3_ai_hardware_controller.yaml',
            description='Controller YAML file under ffw_bringup/config/ffw_a3.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'launch_foot_switch',
            default_value='true',
            description='Whether to launch the foot switch node.',
        ),
        DeclareLaunchArgument(
            'start_zenoh_router',
            default_value='false',
            description=(
                'Start a private rmw_zenohd router only when no shared 1050 '
                'Zenoh router is running. The default reuses the existing router.'
            ),
        ),
        DeclareLaunchArgument(
        'left_gripper_joint',
        default_value='gripper_l_joint1',
        description='Left gripper joint name for gripper_trigger.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    controller_config_file = LaunchConfiguration('controller_config_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    launch_foot_switch = LaunchConfiguration('launch_foot_switch')
    start_zenoh_router = LaunchConfiguration('start_zenoh_router')
    left_gripper_joint = LaunchConfiguration('left_gripper_joint')

    zenoh_router = ExecuteProcess(
        name='rmw_zenohd',
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        additional_env={
            'ZENOH_CONFIG_OVERRIDE': (
                'listen/endpoints=["tcp/127.0.0.1:7447"];'
                'transport/shared_memory/enabled=true'
            ),
        },
        output='both',
        condition=IfCondition(start_zenoh_router),
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_a3',
            controller_config_file,
        ]
    )

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
                [FindPackageShare('ffw_description'), 'urdf', 'ffw_a3', description_file]
            ),
            ' ',
            'use_mock_hardware:=', use_mock_hardware,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller',
            'joystick_controller',
            'joint_state_broadcaster',
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

    foot_switch_node = Node(
        package='ffw_bringup',
        executable='foot_switch_node',
        name='foot_switch_node',
        output='both',
        parameters=[{'save_pose_id': 4}],
        condition=IfCondition(launch_foot_switch),
    )

    gripper_trigger_node = Node(
        package='ffw_joint_trajectory_command_broadcaster',
        executable='gripper_trigger',
        name='gripper_trigger',
        output='both',
        parameters=[{'press_threshold': -0.5, 'left_gripper_joint': left_gripper_joint}],
    )

    shutdown_on_router_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=zenoh_router,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='A3 Zenoh router exited; stopping partial launch.'
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
                        reason='A3 ros2_control_node exited; stopping partial launch.'
                    )
                )
            ],
        )
    )

    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            follower_joint_state_relay_node,
            gripper_trigger_node,
        ]
    )

    delayed_leader = TimerAction(period=1.0, actions=[leader_with_namespace, foot_switch_node])
    delayed_runtime = TimerAction(period=0.25, actions=[zenoh_router, delayed_leader])

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=acquire_single_instance_lock),
            # Match the shared 1050 Cyclo transport even when this launch is
            # started from a plain shell without ROS/Zenoh environment exports.
            SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'),
            SetEnvironmentVariable('ROS_DOMAIN_ID', '30'),
            SetEnvironmentVariable(
                'ZENOH_CONFIG_OVERRIDE',
                'connect/endpoints=["tcp/127.0.0.1:7447"];'
                'transport/shared_memory/enabled=true',
            ),
            shutdown_on_router_exit,
            shutdown_on_control_exit,
            delayed_runtime,
        ]
    )
