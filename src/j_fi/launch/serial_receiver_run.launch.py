from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='j_fi',
            executable='receiver',
            name='receiver',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud_rate': 115200}
            ],
        )
    ])
