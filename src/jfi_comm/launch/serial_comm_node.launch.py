import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
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

    # Get launch configurations.
    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')
    system_id = LaunchConfiguration('system_id')
    component_id = LaunchConfiguration('component_id')

    # Define the Node
    comm_node = Node(
        package='jfi_comm',
        executable='serial_comm_node',
        name='comm_node',
        output='screen',
        parameters=[
            {'port_name': port_name},
            {'baud_rate': baud_rate},
            {'system_id': system_id},
            {'component_id': component_id}
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        system_id_arg,
        component_id_arg,
        comm_node
    ])
