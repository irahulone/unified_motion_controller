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

    # Xacroファイルのパス
    xacro_file = os.path.join(pkg_share, 'src/description/pioneer_multi_robot.xacro')

    # RViz設定ファイルのパス
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    return LaunchDescription([
        # ロボットの台数を指定（デフォルト3台）
        DeclareLaunchArgument("num_robots", default_value="3", description="Number of robots to spawn"),

        # RVizの設定ファイルパスを指定
        DeclareLaunchArgument("rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),

        # Gazeboやシミュレーション用の時間を有効化
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        # robot_state_publisherを起動（Xacroを動的に処理）
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", 
                    xacro_file, 
                    " num_robots:=", 
                    LaunchConfiguration("num_robots")
                ]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }]
        ),

        # RViz2を起動
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
