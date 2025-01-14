from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='j_fi',
            executable='sender',
            name='sender',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud_rate': 115200}
            ],
        )
    ])
