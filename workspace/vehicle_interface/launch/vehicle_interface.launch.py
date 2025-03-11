from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the installed project_utils config directory
    config = os.path.join(
        get_package_share_directory('project_utils'),
        'config',
        'vehicle_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='vehicle_interface',
            executable='vehicle_interface_node',
            name='vehicle_interface_node',
            output='screen',
            parameters=[config]
        )
    ])
