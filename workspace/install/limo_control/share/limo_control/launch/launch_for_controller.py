from launch import LaunchDescription
from launch_ros import actions
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'limo.launch.py' file in the limo_simulation package
    limo_launch_file = os.path.join(
        get_package_share_directory('limo_simulation'),  # Get share directory for limo_simulation
        'launch',  # Directory where the launch files are stored
        'limo.launch.py'  # The specific launch file for the simulation
    )

    return LaunchDescription([
        # Include the limo_simulation launch file to start the simulation
        IncludeLaunchDescription(
            actions.PythonLaunchDescriptionSource(limo_launch_file),
            launch_arguments={}.items()  # No additional arguments needed, but can be added if required
        ),
        # Start the controller node
        actions.Node(
            package='limo_control',
            executable='limo_control',  # This matches the executable for your controller node
            name='controller_node',
            output='screen',
        ),
    ])