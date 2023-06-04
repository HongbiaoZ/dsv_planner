import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_dir_graph_visualization = os.path.join(get_package_share_directory('graph_utils'), 'config', 'default.yaml')
    print(param_dir_graph_visualization)

    return LaunchDescription([
        Node(
            package='graph_utils',
            executable='graph_visualization',
            name='graph_visualization',
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                param_dir_graph_visualization
            ]
         )
    ])


