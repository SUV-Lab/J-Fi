import os

from jinja2 import Environment, FileSystemLoader
from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart, OnProcessExit

def launch_setup(context, *args, **kwargs):

    # socat PTY,link=/dev/virtual_tty1 PTY,link=/dev/virtual_tty2
    socat_process = ExecuteProcess(
        cmd=[FindExecutable(name='socat'), 'PTY,link=/tmp/virtual_tty1', 'PTY,link=/tmp/virtual_tty2'],
        output='screen',
    )

    # Define the Node
    pub_comm_node = Node(
        package='jfi_comm',
        executable='serial_comm_node',
        name='pub_comm_node',
        output='screen',
        parameters=[
            {'port_name': '/tmp/virtual_tty1'},
            {'baud_rate': 57600}
        ]
    )

    # Define the Node
    sub_comm_node = Node(
        package='jfi_comm',
        executable='serial_comm_node',
        name='sub_comm_node',
        output='screen',
        parameters=[
            {'port_name': '/tmp/virtual_tty2'},
            {'baud_rate': 57600}
        ]
    )

    topic_pub = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='hello_publisher',
        output='screen',
        parameters=[{'publish_rate': 1.0}],
        remappings=[
            ('chatter', '/hello_topic')
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    nodes_to_start = [
        socat_process,
        pub_comm_node,
        sub_comm_node,
        topic_pub,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription( declared_arguments + [OpaqueFunction(function=launch_setup)])