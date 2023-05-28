import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    param_dir_graph_planner = os.path.join(get_package_share_directory('graph_planner'), 'config', 'default.yaml')
    print(param_dir_graph_planner)

    return LaunchDescription([
        Node(
            package='graph_planner',
            executable='graph_planner_exe',
            name='graph_planner',
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                param_dir_graph_planner
            ]
         ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
              os.path.join(get_package_share_directory('graph_utils'), 'launch/graph_visualization.py'
        	    )
            )
        )
    ])