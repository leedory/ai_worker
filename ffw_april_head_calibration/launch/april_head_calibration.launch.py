#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/rgb_gray/image_rect_gray',
            description='sensor_msgs/Image topic.',
        ),
        DeclareLaunchArgument(
            'joint_trajectory_topic',
            default_value='/head_controller/joint_trajectory',
            description=(
                'Remapped to /leader/joystick_controller_left/joint_trajectory (FFW).'
            ),
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='JointState with head joints.',
        ),
        DeclareLaunchArgument(
            'target_marker_id',
            default_value='-1',
            description='AprilTag id, or -1 for largest tag.',
        ),
        DeclareLaunchArgument(
            'apriltag_family',
            default_value='tag36h11',
            description='AprilTag family (OpenCV): tag36h11, tag36h10, tag25h9, tag16h5.',
        ),
        DeclareLaunchArgument(
            'image_transport_encoding',
            default_value='auto',
            description='auto = use Image.msg.encoding (ZED gray/mono8).',
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish overlaid debug image.',
        ),
        DeclareLaunchArgument(
            'publish_rviz_markers',
            default_value='true',
            description='Publish MarkerArray for RViz2.',
        ),
        DeclareLaunchArgument(
            'head_joint1_pulse_reference',
            default_value='257',
            description='Reference pulse for head_joint1 (delta = current − ref).',
        ),
        DeclareLaunchArgument(
            'head_joint2_pulse_reference',
            default_value='-20',
            description='Reference pulse for head_joint2.',
        ),
        DeclareLaunchArgument(
            'head_joint_pulse_resolution',
            default_value='2048',
            description=(
                'Encoder pulses per revolution for rad→pulse conversion '
                '(logging and save_delta YAML).'
            ),
        ),
        DeclareLaunchArgument(
            'save_delta_yaml_path',
            default_value=(
                '~/ros2_ws/src/ai_worker/ffw_april_head_calibration/config/'
                'ffw_head_joint_pulse_delta.yaml'
            ),
            description=(
                'YAML path for save_delta Trigger; empty = ~/.ros/ffw_head_joint_pulse_delta.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'enable_lost_tag_search',
            default_value='true',
            description='Sweep head pitch when no tag (min/max/step use node defaults).',
        ),
        DeclareLaunchArgument(
            'lost_tag_miss_grace_frames',
            default_value='15',
            description=(
                'After a good track, hold last command for this many consecutive '
                'miss frames before pitch sweep (0 = immediate sweep).'
            ),
        ),
        DeclareLaunchArgument(
            'apriltag_detection_max_side',
            default_value='0',
            description='Max image long edge before downscale; 0 = full resolution.',
        ),
        DeclareLaunchArgument(
            'run_apply_head_homing_offset',
            default_value='false',
            description=(
                'If true, also run apply_head_homing_offset.launch.py once at startup '
                '(patches Homing Offset in ros2_control xacro from delta YAML).'
            ),
        ),
        DeclareLaunchArgument(
            'ros2_control_xacro_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_description'),
                'ros2_control',
                'ffw_sg2_rev1_follower',
                'ffw_sg2_follower.ros2_control.xacro',
            ]),
            description=(
                'Target xacro when run_apply_head_homing_offset is true '
                '(same as apply_head_homing_offset.launch.py).'
            ),
        ),
        DeclareLaunchArgument(
            'apply_homing_offset_dry_run',
            default_value='false',
            description='Passed to apply_head_homing_offset when run_apply_head_homing_offset is true.',
        ),
        DeclareLaunchArgument(
            'apply_homing_offset_use_backup',
            default_value='false',
            description='Passed to apply_head_homing_offset when run_apply_head_homing_offset is true.',
        ),
        Node(
            package='ffw_april_head_calibration',
            executable='april_head_calibration',
            name='april_head_calibration',
            output='screen',
            remappings=[
                (
                    '/head_controller/joint_trajectory',
                    '/leader/joystick_controller_left/joint_trajectory',
                ),
            ],
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'joint_trajectory_topic': LaunchConfiguration(
                    'joint_trajectory_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'target_marker_id': ParameterValue(
                    LaunchConfiguration('target_marker_id'),
                    value_type=int,
                ),
                'apriltag_family': LaunchConfiguration('apriltag_family'),
                'image_transport_encoding': LaunchConfiguration(
                    'image_transport_encoding'),
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('publish_debug_image'),
                    value_type=bool,
                ),
                'publish_rviz_markers': ParameterValue(
                    LaunchConfiguration('publish_rviz_markers'),
                    value_type=bool,
                ),
                'head_joint1_pulse_reference': ParameterValue(
                    LaunchConfiguration('head_joint1_pulse_reference'),
                    value_type=int,
                ),
                'head_joint2_pulse_reference': ParameterValue(
                    LaunchConfiguration('head_joint2_pulse_reference'),
                    value_type=int,
                ),
                'head_joint_pulse_resolution': ParameterValue(
                    LaunchConfiguration('head_joint_pulse_resolution'),
                    value_type=int,
                ),
                'save_delta_yaml_path': LaunchConfiguration(
                    'save_delta_yaml_path'),
                'ros2_control_xacro_path': LaunchConfiguration(
                    'ros2_control_xacro_path'),
                'enable_lost_tag_search': ParameterValue(
                    LaunchConfiguration('enable_lost_tag_search'),
                    value_type=bool,
                ),
                'lost_tag_miss_grace_frames': ParameterValue(
                    LaunchConfiguration('lost_tag_miss_grace_frames'),
                    value_type=int,
                ),
                'apriltag_detection_max_side': ParameterValue(
                    LaunchConfiguration('apriltag_detection_max_side'),
                    value_type=int,
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ffw_april_head_calibration'),
                    'launch',
                    'apply_head_homing_offset.launch.py',
                ]),
            ),
            launch_arguments=[
                ('delta_yaml_path', LaunchConfiguration('save_delta_yaml_path')),
                ('ros2_control_xacro_path', LaunchConfiguration(
                    'ros2_control_xacro_path')),
                ('dry_run', LaunchConfiguration('apply_homing_offset_dry_run')),
                ('use_backup', LaunchConfiguration('apply_homing_offset_use_backup')),
            ],
            condition=IfCondition(LaunchConfiguration('run_apply_head_homing_offset')),
        ),
    ])
