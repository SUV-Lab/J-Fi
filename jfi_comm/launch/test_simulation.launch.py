import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    socat_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="socat"),
            "PTY,link=/tmp/virtual_tty1,raw,echo=0",
            "PTY,link=/tmp/virtual_tty2,raw,echo=0",
        ],
        output="screen",
    )

    comm_node_1 = Node(
        package="jfi_comm",
        executable="serial_comm_node",
        namespace="pc1",
        name="serial_comm_node",
        output="screen",
        parameters=[{"port_name": "/tmp/virtual_tty1", "system_id": 1}],
    )
    # evaluator_1 = Node(
    #     package="jfi_comm",
    #     executable="evaluator_node",
    #     namespace="pc1",
    #     name="evaluator_node",
    #     output="screen",
    #     parameters=[{"my_system_id": 1}]
    # )

    comm_node_2 = Node(
        package="jfi_comm",
        executable="serial_comm_node",
        namespace="pc2",
        name="serial_comm_node",
        output="screen",
        parameters=[{"port_name": "/tmp/virtual_tty2", "system_id": 2}],
    )
    # evaluator_2 = Node(
    #     package="jfi_comm",
    #     executable="evaluator_node",
    #     namespace="pc2",
    #     name="evaluator_node",
    #     output="screen",
    #     parameters=[{"my_system_id": 2}]
    # )

    # delayed_evaluator_1 = TimerAction(
    #     period=2.0,
    #     actions=[evaluator_1]
    # )
    # delayed_evaluator_2 = TimerAction(
    #     period=2.0,
    #     actions=[evaluator_2]
    # )

    return LaunchDescription(
        [
            socat_process,
            comm_node_1,
            comm_node_2,
            # delayed_evaluator_1,
            # delayed_evaluator_2
        ]
    )
