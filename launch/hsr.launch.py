#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def declare_arguments():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='hsrb_description',
            description='Description package with robot URDF/xacro files.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='hsrb4s.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'openmm_rgbd_compression',
            default_value='true',
            description='Publish OpenMM-compatible compressed RGB-D topics.',
        )
    )
    # Isaac Sim 側 (この ros2 コンテナ) の MoveIt 付属 RViz2 を起動するか。
    # 既定 false: ふだん RViz は別 (Singularity 側等) で立てるので二重起動を避ける。
    # 立てたいときだけ `ros2 launch /hsr.launch.py use_rviz:=true`。
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch the MoveIt RViz2 on the Isaac Sim (ros2 container) side.',
        )
    )
    return declared_arguments


def render_xacro_and_launch_robot_state_publisher(context: LaunchContext, args: dict) -> str:
    robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory(
                context.perform_substitution(args['description_package'])),
            'robots',
            context.perform_substitution(args['description_file']),
        ),
        mappings={
            'gazebo_sim': 'False',
            'rviz_sim': 'False',
        },
    ).toxml()
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content},
            ],
            remappings=[('joint_states', '/whole_body/joint_states')],
            output={'both': 'log'},
        )
    ]


def generate_launch_description():
    args = {}
    for arg in declare_arguments():
        args[arg.name] = LaunchConfiguration(arg.name)

    relay_node = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    'hsrb_relay_topics.launch.xml',
                ]),
            )
        ]
    )

    sensor_frames = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    'hsrb_sensor_frames.launch.xml',
                ]),
            )
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'source_list': ['/joint_states']},
        ],
        namespace='whole_body',
        remappings=[('robot_description', '/robot_description')],
    )

    robot_state_publisher = OpaqueFunction(
        function=render_xacro_and_launch_robot_state_publisher, args=[args]
    )

    common = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('hsrb_common_launch'),
                        'launch',
                        'hsrb_common.launch.py',
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    # false: HSR 純正 localizer/nav (laser_2d_localizer, pose_integrator)
                    # を起動しない。自己位置推定は Singularity 側の emcl2/pumas に任せる
                    # (本番同等)。robot_state_publisher / odom / 知覚は下で別途上げるので残る。
                    # HSR 純正ナビを使いたいときだけ 'true' に戻す。
                    'use_navigation': 'false',
                    'map': os.path.join(
                        get_package_share_directory('tmc_wrs_gazebo_worlds'),
                        'maps',
                        'wrs2020',
                        'map.yaml',
                    ),
                    'use_manipulation': 'true',
                    'use_teleop': 'false',
                    'use_joy_node': 'false',
                }.items(),
            ),
        ]
    )

    moveit = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('hsrb_moveit_config'),
                        'launch',
                        'hsrb_demo.launch.py',
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'use_rviz': LaunchConfiguration('use_rviz'),
                }.items(),
            ),
        ]
    )

    # task_evaluators = GroupAction(
    #    actions=[
    #        IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource(
    #                [
    #                    os.path.join(
    #                        get_package_share_directory('tmc_gazebo_task_evaluators'),
    #                        'launch',
    #                        'robocup2021.launch.py',
    #                    )
    #                ]
    #            ),
    #            launch_arguments={
    #                'use_sim_time': 'true'
    #            }.items(),
    #        ),
    #    ]
    # )

    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(
                'hsrb_bringup'), 'launch', 'odoms.py')
        ])
    )

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    local_republisher = os.path.join(
        os.path.dirname(launch_dir),
        'scripts',
        'openmm_rgbd_compression_republisher.py',
    )
    image_republisher_script = (
        local_republisher
        if os.path.exists(local_republisher)
        else '/openmm_rgbd_compression_republisher.py'
    )
    openmm_rgbd_compression = ExecuteProcess(
        cmd=[
            'python3',
            image_republisher_script,
            '--ros-args',
            '-p', 'use_sim_time:=true',
        ],
        output='screen',
        condition=IfCondition(args['openmm_rgbd_compression']),
    )

    nodes = [
        relay_node,
        openmm_rgbd_compression,
        sensor_frames,
        joint_state_publisher,
        robot_state_publisher,
        common,
        moveit,
        odom,
        # task_evaluators
    ]

    return LaunchDescription(
        declare_arguments()
        + [
            SetParameter(name='use_sim_time', value=True),
        ]
        + nodes
    )
