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

    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')

    # Define the Node
    comm_node = Node(
        package='serial_comm',
        executable='serial_comm_node',
        name='comm_node',
        output='screen',
        parameters=[
            {'port_name': port_name},
            {'baud_rate': baud_rate}
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        comm_node
    ])
