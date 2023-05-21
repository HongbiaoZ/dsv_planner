import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    param_dir_planning = os.path.join(get_package_share_directory('davp_launch'), 'config', 'exploration_indoor.yaml')
    param_dir_octomap = os.path.join(get_package_share_directory('davp_launch'), 'config', 'octomap_indoor.yaml')
    param_dir_boundary = os.path.join(get_package_share_directory('davp_launch'), 'data', 'boundary.ply')
    rviz_path = os.path.join(get_package_share_directory('davp_launch'), 'default.rviz')
    use_boundary = LaunchConfiguration('use_boundary', default='false')
    enable_bag_record = LaunchConfiguration('enable_bag_record', default='false')
    bag_name = LaunchConfiguration('bag_name', default='indoor')
    print(param_dir_planning, param_dir_octomap)

    return LaunchDescription([
        Node(
            package='dsvp_launch',
            executable='exploration',
            name='exploration',
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                param_dir_planning
            ]
         ),
         Node(
            package='dsvplanner',
            executable='dsvplanner',
            name='dsvplanner',
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                param_dir_planning, param_dir_octomap
            ]
         ),
         Node(
            package='dsvp_launch',
            executable='navigationBoundary',
            name='navigationBoundary',
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
               {'boundary_file_dir': param_dir_boundary},
               {'sendBoundary': 'true'},
               {'sendBoundaryInterval': '2'}
            ],
            condition=IfCondition(use_boundary)
         ),
         Node(
            package='rviz2',
            executable='rviz2',
            name='dsvp_rviz',
            prefix=['stdbuf -o L'],
            output='screen',
            arguments=['-d', rviz_path]
         ),
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/rosbag_record.py']
            ),
            condition=IfCondition(enable_bag_record),
            launch_arguments={'bag_name': bag_name}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
              os.path.join(get_package_share_directory('graph_planner'), 'launch/graph_planner.py'
        	    )
            )
        )
    ])