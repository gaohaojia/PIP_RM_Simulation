#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

class RobotInitialPosition:
    robot_name = ['Hero', 'Engineer', 'Infantry1', 'Infantry2', 'Infantry3', 'Sentry']
    _RMUC = {
        'Hero': ['2.5', '9.0', '0.2', '0.0'],
        'Engineer': ['3.5', '7.5', '0.2', '0.0'],
        'Infantry1': ['3.0', '6.0', '0.2', '0.0'],
        'Infantry2': ['2.5', '6.0', '0.2', '0.0'],
        'Infantry3': ['3.0', '9.0', '0.2', '0.0'],
        'Sentry': ['6.5', '7.5', '0.2', '0.0'],
    }
    position = {
        'RMUC2024_world': _RMUC,
    }


def robot_description(context: LaunchContext, robot_count, world_name, use_sim_time):
    action_list = []
    world_name_str = context.perform_substitution(world_name)
    for i in range(int(context.perform_substitution(robot_count))):
        robot_description = Command([
            'xacro ', 
            os.path.join(
                get_package_share_directory('simulation_bringup'), 
                'urdf', 
                'sensebeetle.xacro'
            )
        ])
        
        start_joint_state_publisher_cmd = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace='robot_{}'.format(i),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }],
            output='screen'
        )

        start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='robot_{}'.format(i),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description
            }],
            output='screen'
        )

        robot_name = RobotInitialPosition.robot_name[i]
        start_spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot_{}'.format(i),
                '-topic', 'robot_{}/robot_description'.format(i),
                '-robot_namespace', 'robot_{}'.format(i),
                '-x', RobotInitialPosition.position[world_name_str][robot_name][0],
                '-y', RobotInitialPosition.position[world_name_str][robot_name][1],
                '-z', RobotInitialPosition.position[world_name_str][robot_name][2],
                '-Y', RobotInitialPosition.position[world_name_str][robot_name][3]
            ],
        )

        action_list.append(start_joint_state_publisher_cmd)
        action_list.append(start_robot_state_publisher_cmd)
        action_list.append(start_spawn_entity_node)

    return action_list

def communication_client_launch(context: LaunchContext, robot_count):
    action_list = []
    for i in range(int(context.perform_substitution(robot_count))):
        communication_client_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('communication_client'), 
                    'launch', 
                    'communication_client.launch.py'
                )
            ),
            launch_arguments={
                'robot_id': str(i),
            }.items()
        )
        action_list.append(SetEnvironmentVariable('ROS_DOMAIN_ID', str(i+1)))
        action_list.append(communication_client_launch)
    action_list.append(SetEnvironmentVariable('ROS_DOMAIN_ID', str(0)))
    return action_list

def world_launch(context: LaunchContext, world_name):
    name = context.perform_substitution(world_name)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'), 
                'launch', 
                'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory('simulation_bringup'), 
                'world', 
                name, 
                name + '.world'
            )
        }.items(),
    )
    return [world_launch]


def generate_launch_description():

    robot_count = LaunchConfiguration('robot_count')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')

    declare_robot_count = DeclareLaunchArgument(
        'robot_count', 
        default_value='3', 
        description='Robot count'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='RMUC2024_world',
        description='Choose world'
    )

    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'), 
                'launch', 
                'gzclient.launch.py'
            )
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_count)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(gazebo_client_launch)
    ld.add_action(OpaqueFunction(function=world_launch, args=[world_name]))
    ld.add_action(OpaqueFunction(function=robot_description, args=[robot_count, world_name, use_sim_time]))
    ld.add_action(OpaqueFunction(function=communication_client_launch, args=[robot_count]))

    return ld