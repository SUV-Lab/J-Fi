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
            # Input Topics
            ("jfi_comm/in/string", "node1/send_string"),
            ("jfi_comm/in/trajectory", "node1/send_trajectory"),
            # Output Topics
            ("jfi_comm/out/string", "node1/received_string"),
            ("jfi_comm/out/trajectory", "node1/received_trajectory"),
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
            # Input Topics
            ("jfi_comm/in/string", "node2/send_string"),
            ("jfi_comm/in/trajectory", "node2/send_trajectory"),
            # Output Topics
            ("jfi_comm/out/string", "node2/received_string"),
            ("jfi_comm/out/trajectory", "node2/received_trajectory"),
        ],
    )

    evaluator = Node(
        package="jfi_comm",
        executable="evaluator_node",
        name="evaluator",
        output="screen",
    )

    nodes_to_start = [
        socat_process,
        node1,
        node2,
        evaluator,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
