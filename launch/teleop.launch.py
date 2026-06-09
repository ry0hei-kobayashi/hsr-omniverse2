#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Isaac Sim 側（ros2 コンテナ）で動かすテレオペ "ロジック本体"。
#
# 役割分担（実機と同じ）:
#   - 前半 (joy_node + teleop_steel_series → /joy): Singularity 側の通常 bringup が担当。
#     Sim のためにこちらを変更する必要はない。
#   - 後半 (このファイル): /joy を解釈してロボットを動かすノード群。実機ではロボット
#     オンボードで動くため、その代わりである Sim 側 (ros2 コンテナ) で動かす。
#
# Sim 固有の差分は 2 点だけ:
#   (1) 台車速度を /omni_base_controller/cmd_vel へ直接送る (Sim は速度 mux を持たない)。
#   (2) pseudo controller に arm/head の joints を明示する。Isaac Sim の
#       arm_trajectory_controller 等は独立 ROS2 ノードではなく isaac_sim_hsr 内の
#       エミュレーションで、パラメータサービスから joints を取得できないため、
#       明示しないと "Communication with arm_trajectory_controller failed." で落ちる。
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Isaac Sim が公開している各 controller の関節（hsr.py の controlled_joints と一致）。
ARM_JOINTS = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
              'wrist_flex_joint', 'wrist_roll_joint']
HEAD_JOINTS = ['head_pan_joint', 'head_tilt_joint']


def declare_arguments():
    return [
        DeclareLaunchArgument('description_package', default_value='hsrb_description'),
        DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='omni_base_controller/wheel_odom'),
        DeclareLaunchArgument('hand_close_force', default_value='0.8'),
    ]


def generate_launch_description():
    # robot_description (pseudo controller が URDF を必要とする)。
    robot_description_content = xacro.process_file(
        os.path.join(get_package_share_directory('hsrb_description'),
                     'robots', 'hsrb4s.urdf.xacro'),
        mappings={'gazebo_sim': 'False', 'rviz_sim': 'False'},
    ).toxml()
    robot_description = {'robot_description': robot_description_content}

    common_launch = FindPackageShare('hsrb_common_launch')

    # 後半①: /joy を解釈して各コマンドへ変換する本体。
    #   台車だけ Sim が待ち受ける /omni_base_controller/cmd_vel へ remap。
    #   操作割り当ては実機標準の config をそのまま使う（/joy は Singularity 側の
    #   teleop_steel_series が並べ替えた 19ボタン配列なので button11/12 も有効）。
    joystick_control_node = Node(
        package='hsrb_joystick_teleop',
        executable='joystick_control_node',
        output='screen',
        emulate_tty=True,
        remappings=[('command_velocity', '/omni_base_controller/cmd_vel')],
        parameters=[
            PathJoinSubstitution([common_launch, 'config', 'joystick_control_config.yaml']),
            {'hand_close_force': LaunchConfiguration('hand_close_force')},
        ],
    )

    # 後半②: 手先(エンドエフェクタ)速度 → アーム軌道。
    pseudo_ee_controller_node = Node(
        package='hsrb_pseudo_endeffector_controller',
        executable='hsrb_pseudo_endeffector_controller',
        name='pseudo_endeffector_controller',
        parameters=[
            robot_description,
            PathJoinSubstitution([common_launch, 'config',
                                  'pseudo_endeffector_controller_config.yaml']),
            # Sim 用に arm の joints を明示（パラメータサービス問い合わせを回避）。
            {'arm_trajectory_controller': {'joints': ARM_JOINTS}},
        ],
        remappings=[('odom', LaunchConfiguration('odom_topic'))],
    )

    # 後半③: 各関節の速度指令 → 各 controller の軌道。
    pseudo_velocity_controller_node = Node(
        package='tmc_pseudo_velocity_controller',
        executable='pseudo_velocity_controller',
        name='pseudo_velocity_controller',
        parameters=[
            robot_description,
            PathJoinSubstitution([common_launch, 'config',
                                  'pseudo_velocity_controller_config.yaml']),
            # Sim 用に arm/head の joints を明示（gripper は config 側で明示済み）。
            {'arm_trajectory_controller': {'joints': ARM_JOINTS},
             'head_trajectory_controller': {'joints': HEAD_JOINTS}},
        ],
    )

    return LaunchDescription(
        declare_arguments()
        + [
            joystick_control_node,
            pseudo_ee_controller_node,
            pseudo_velocity_controller_node,
        ]
    )
