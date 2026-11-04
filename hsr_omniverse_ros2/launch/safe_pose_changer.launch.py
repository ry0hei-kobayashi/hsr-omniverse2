#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Isaac Sim 用の safe_pose_changer。実機用パッケージは変更しない。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Isaac Sim の robot/hsr/robot.py が公開する controller の関節と一致させる。
ARM_JOINTS = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
              'wrist_flex_joint', 'wrist_roll_joint']
HEAD_JOINTS = ['head_pan_joint', 'head_tilt_joint']
GRIPPER_JOINTS = ['hand_motor_joint']


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tmc_safe_pose_changer',
            executable='safe_pose_changer',
            # __node の全体 remap は内部の action 用ノードまで同名にしてしまうため、name は指定しない
            parameters=[
                os.path.join(
                    get_package_share_directory('hsrb_manipulation_launch'),
                    'config', 'hsrb_joint_limits.yaml'),
                {'wait_for_controller_milliseconds': 60000},
                {'controllers': ['arm_trajectory_controller',
                                 'head_trajectory_controller',
                                 'gripper_controller']},
                # Sim 用に joints を明示（teleop.launch.py と同じ方式）。
                # Isaac のサービス起動待ちで失敗し、gripper が未宣言になるのを防ぐ。
                {'arm_trajectory_controller': {'joints': ARM_JOINTS},
                 'head_trajectory_controller': {'joints': HEAD_JOINTS},
                 'gripper_controller': {'joints': GRIPPER_JOINTS}},
            ],
        ),
    ])
