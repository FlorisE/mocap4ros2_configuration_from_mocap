import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory('configuration_from_mocap'), 'config', 'marker_sim_params.yaml')
    return LaunchDescription([
        Node(
            package='configuration_from_mocap',
            executable='marker_sim_main',
            name='marker_sim_main',
            parameters=[params_file_path],
        ),
    ])
