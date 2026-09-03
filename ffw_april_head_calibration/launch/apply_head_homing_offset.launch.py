#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Run ffw_apply_head_homing_offset_from_delta with paths from launch arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *_args, **_kwargs):
    delta = context.perform_substitution(LaunchConfiguration('delta_yaml_path'))
    xacro = context.perform_substitution(LaunchConfiguration('ros2_control_xacro_path'))
    dry_raw = context.perform_substitution(LaunchConfiguration('dry_run')).lower()
    backup_raw = context.perform_substitution(LaunchConfiguration('use_backup')).lower()
    dry = dry_raw in ('true', '1', 'yes', 'on')
    backup = backup_raw in ('true', '1', 'yes', 'on')

    cmd = [
        'ros2',
        'run',
        'ffw_april_head_calibration',
        'ffw_apply_head_homing_offset_from_delta',
        '--',
        '--delta-yaml',
        delta,
        '--xacro',
        xacro,
    ]
    if backup:
        cmd.append('--backup')
    if dry:
        cmd.append('--dry-run')

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delta_yaml_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_april_head_calibration'),
                'config',
                'ffw_head_joint_pulse_delta.yaml',
            ]),
            description='ffw_head_joint_pulse_delta.yaml from save_delta.',
        ),
        DeclareLaunchArgument(
            'ros2_control_xacro_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_description'),
                'ros2_control',
                'ffw_bg2_rev4_follower',
                'ffw_bg2_follower.ros2_control.xacro',
            ]),
            description=(
                'ros2_control xacro to patch (Homing Offset for head DXL gpios).'
            ),
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, print planned offsets only; do not write the xacro.',
        ),
        DeclareLaunchArgument(
            'use_backup',
            default_value='false',
            description='If true, write a .bak copy of the xacro before overwriting.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
