from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージ名と呼び出すLaunchファイルのパスを取得
    pkg_name = 'rover_description'
    display_launch_file = os.path.join(get_package_share_directory(pkg_name), 'launch', 'display.launch.py')
    teleop_launch_file = os.path.join(get_package_share_directory("teleop_core"), 'gui.launch.py')
    
    return LaunchDescription([
        # 別のLaunchファイルを含める
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_file)
        )
    ])
