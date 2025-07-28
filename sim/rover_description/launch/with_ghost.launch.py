from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'rover_description'
    pioneer_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'pioneer.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p2',
                'x': '-2.0', 
                'y': '0.0',
                't': '0.0',
                'hw': 'hw',
                'a': '0.2'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p3',
                'x': '2.0',
                'y': '0.0', 
                't': '1.0',
                'hw': 'hw',
                'a': '0.2'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p4',
                'x': '0.0', 
                'y': '3.0',
                't': '-1.0',
                'hw': 'hw',
                'a': '0.2'
            }.items()
        ),
        Node(
            package='reference_srv',
            executable='gps_reference_server',
            name='gps_reference_server',
            output='screen',
        )
    ])
