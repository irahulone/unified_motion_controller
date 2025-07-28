import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'rover_description'
    display_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'display.launch.py')
    cluster_launch_file = os.path.join(get_package_share_directory("cluster_node"), 'cluster_virtjoy.launch.py')
    #pioneers_launch_file = os.path.join(get_package_share_directory(pkg_name), 'with_ghost.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_launch_file)
        ),
    ])