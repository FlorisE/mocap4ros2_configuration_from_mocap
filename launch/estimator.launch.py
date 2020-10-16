import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file_path = os.path.join(get_package_share_directory('configuration_from_mocap'), 'config', 'estimator_params.yaml')
    return LaunchDescription([
        Node(
            package='configuration_from_mocap',
            executable='estimator_main',
            name='estimator_main',
            parameters=[params_file_path],
            arguments=[('--ros-args --log-level DEBUG')],
        ),
    ])
