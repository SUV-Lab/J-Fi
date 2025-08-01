import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # Create two virtual serial ports using socat and connect them to each other.
    socat_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="socat"),
            "PTY,link=/tmp/virtual_tty1,raw,echo=0",
            "PTY,link=/tmp/virtual_tty2,raw,echo=0",
        ],
        output="screen",
        shell=True,
    )

    node1 = Node(
        package="jfi_comm",
        executable="serial_comm_node",
        name="node1",
        output="screen",
        parameters=[
            {"port_name": "/tmp/virtual_tty1"},
            {"baud_rate": 115200},
            {"system_id": 1},
            {"component_id": 1},
        ],
        remappings=[
            ("jfi_comm/out/string", "node1/received_string"),
        ],
    )

    node2 = Node(
        package="jfi_comm",
        executable="serial_comm_node",
        name="node2",
        output="screen",
        parameters=[
            {"port_name": "/tmp/virtual_tty2"},
            {"baud_rate": 115200},
            {"system_id": 2},
            {"component_id": 2},
        ],
        remappings=[
            ("jfi_comm/out/string", "node2/received_string"),
        ],
    )

    nodes_to_start = [
        socat_process,
        node1,
        node2,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
