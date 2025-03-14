from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the controller node
        Node(
            package='limo_control',
            executable='limo_control',  # This matches the executable for your controller node
            name='controller_node',
            output='screen',
        ),
    ])