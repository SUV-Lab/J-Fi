import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def create_comm_node(context, *args, **kwargs):
    port_name_str    = context.perform_substitution(LaunchConfiguration('port_name'))
    baud_rate_str    = context.perform_substitution(LaunchConfiguration('baud_rate'))
    system_id_str    = context.perform_substitution(LaunchConfiguration('system_id'))
    component_id_str = context.perform_substitution(LaunchConfiguration('component_id'))

    baud_rate    = int(baud_rate_str)
    system_id    = int(system_id_str)
    component_id = int(component_id_str)

    send_topic = f'from_vehicle{system_id_str}/planning/broadcast_traj_send'
    recv_topic = f'to_vehicle{system_id_str}/planning/broadcast_traj_recv'

    comm_node = Node(
        package='jfi_comm',
        executable='serial_comm_node',
        name='comm_node',
        output='screen',
        parameters=[
            {'port_name': port_name_str},
            {'baud_rate': baud_rate},
            {'system_id': system_id},
            {'component_id': component_id},
        ],
        remappings=[
            ('/planning/broadcast_traj_send', send_topic),
            ('/planning/broadcast_traj_recv', recv_topic),
        ]
    )
    return [comm_node]

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Serial port device path'
    )
    baud_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial port baud rate'
    )
    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='1',
        description='MAVLink system ID'
    )
    component_id_arg = DeclareLaunchArgument(
        'component_id',
        default_value='1',
        description='MAVLink component ID'
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        system_id_arg,
        component_id_arg,
        OpaqueFunction(function=create_comm_node),
    ])
