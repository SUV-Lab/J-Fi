import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import TimerAction

# 런치 파일의 필수 진입점 함수
def generate_launch_description():
    
    # 1. 가상 시리얼 포트를 생성하는 socat 프로세스
    socat_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="socat"),
            "PTY,link=/tmp/virtual_tty1,raw,echo=0",
            "PTY,link=/tmp/virtual_tty2,raw,echo=0",
        ],
        output="screen"
    )

    # 2. 가상 PC 1을 위한 노드 그룹 (네임스페이스: 'pc1')
    comm_node_1 = Node(
        package="jfi_comm",
        executable="serial_comm_node",
        namespace="pc1",
        name="serial_comm_node",
        output="screen",
        parameters=[{"port_name": "/tmp/virtual_tty1", "system_id": 1}]
    )
    evaluator_1 = Node(
        package="jfi_comm",
        executable="evaluator_node",
        namespace="pc1",
        name="evaluator_node",
        output="screen",
        parameters=[{"my_system_id": 1}]
    )

    # 3. 가상 PC 2를 위한 노드 그룹 (네임스페이스: 'pc2')
    comm_node_2 = Node(
        package="jfi_comm",
        executable="serial_comm_node",
        namespace="pc2",
        name="serial_comm_node",
        output="screen",
        parameters=[{"port_name": "/tmp/virtual_tty2", "system_id": 2}]
    )
    evaluator_2 = Node(
        package="jfi_comm",
        executable="evaluator_node",
        namespace="pc2",
        name="evaluator_node",
        output="screen",
        parameters=[{"my_system_id": 2}]
    )

    delayed_evaluator_1 = TimerAction(
        period=2.0,
        actions=[evaluator_1]
    )
    delayed_evaluator_2 = TimerAction(
        period=2.0,
        actions=[evaluator_2]
    )

    # 4. 실행할 모든 노드와 프로세스를 담은 LaunchDescription 객체를 반환
    return LaunchDescription([
        socat_process,
        comm_node_1,
        comm_node_2,
        delayed_evaluator_1,
        delayed_evaluator_2
    ])