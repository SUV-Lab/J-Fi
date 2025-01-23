import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 인자 선언
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

    # 런치 인자 읽기
    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')

    # Node 정의
    comm_node = Node(
        package='serial_comm',           # 빌드된 패키지 이름
        executable='serial_comm_node',       # 실행 파일 이름 (comm_node)
        name='comm_node',                    # 노드 이름
        output='screen',
        parameters=[
            {'port_name': port_name},
            {'baud_rate': baud_rate}
        ]
    )

    # LaunchDescription 반환
    return LaunchDescription([
        port_arg,
        baud_arg,
        comm_node
    ])
