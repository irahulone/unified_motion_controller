import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージディレクトリを取得
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')

    # RViz設定ファイルのパス
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/cluster3withhw.rviz')

    return LaunchDescription([
        # RVizの設定ファイルパスを指定
        DeclareLaunchArgument("rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),

        # Gazeboやシミュレーション用の時間を有効化
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        # RViz2を起動
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
        Node(
            package="register_service",
            executable="register_service",
            name="robot_register_server",
            output="screen",
        ),
        Node(
            package="fake_rover_state_controller",
            executable="rover_sim",
            name="rover_sim",
            output="screen",
        ),
    ])
