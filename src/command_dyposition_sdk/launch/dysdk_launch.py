rom launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='command_dyposition_sdk',
            executable='server',
            name='dynserver_node',
            output='screen',
            parameters=[
                {"port_name": "/dev/ttyUSB0"},
                {"baud_rate": 115200},
                {"dxl_id": 1},
            ]
        ),
        
    ])
