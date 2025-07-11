from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboclaw2_joy',
            executable='roboclaw2',
            name='roboclaw_joy_node',
            output='screen',
            parameters=[
                {"port": "/dev/ttyACM1"},
                {"baud_rate": 115200},
            ]
        ),
    
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        
    ])