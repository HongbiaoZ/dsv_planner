import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    bag_name = LaunchConfiguration('bag_name', default='dsvp_garage')
    bag_file = os.path.join(EnvironmentVariable('HOME'), 'Desktop/simulation_bags/dsvp', 'dsvp_garage' + bag_name)

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rosbag', 'record', '-o', bag_file, '/tf /registered_scan /state_estimation /way_point /local_graph /global_graph /new_tree_path /remaining_tree_path /planner_boundry /globalSelectedfrontier /threefrontier /graph_planner_command /graph_planner_status /graph_planner_path /global_frontier /local_frontier /threefrontier /plan_time /runtime /totaltime /next_goal /dsvplanner/octomap_occupied'],
            output="screen"
        )
    ])