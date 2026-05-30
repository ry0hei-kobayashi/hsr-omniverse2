#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
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
                launch_arguments={'use_sim_time': 'true'}.items(),
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

    nodes = [
        relay_node,
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
