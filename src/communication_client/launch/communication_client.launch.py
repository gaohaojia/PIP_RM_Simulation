import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def integration_service_launch(context: LaunchContext, robot_id):
    id = context.perform_substitution(robot_id)
    integration_service = ExecuteProcess(
        cmd=['integration-service', os.path.join(get_package_share_directory('simulation_bringup'), 'yaml', 'robot_{}.yaml'.format(id))],
        output='screen'
    )
    return [integration_service]

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    network_port = LaunchConfiguration('network_port')
    network_ip = LaunchConfiguration('network_ip')
    
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    declare_network_port = DeclareLaunchArgument('network_port', default_value='12131', description='')
    declare_network_ip = DeclareLaunchArgument('network_ip', default_value='192.168.31.207', description='')

    communication_client_node = Node(
        package='communication_client',
        executable='communication_client_node',
        name='communication_client',
        output='screen',
        respawn=True,
        parameters=[{
            'robot_id': robot_id,
            'network_port': network_port,
            'network_ip': network_ip
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_id)
    ld.add_action(declare_network_port)
    ld.add_action(declare_network_ip)

    ld.add_action(OpaqueFunction(function=integration_service_launch, args=[robot_id]))
    
    ld.add_action(communication_client_node)
    
    return ld