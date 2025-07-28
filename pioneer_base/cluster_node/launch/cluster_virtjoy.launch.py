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
    ld = LaunchDescription()

    # Get the directory of this launch file
    package_name = 'cluster_node'
    pkg_share = get_package_share_directory(package_name)
    # Construct paths to the parameter files relative to the launch file directory
    cluster_file = os.path.join(pkg_share, 'config', 'cluster_multi.yaml')

    # Check if parameter files exist
    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    # Nodes
    run_cluster_node = Node(
        package="cluster_node",
        executable="cluster_controller",
        parameters=[cluster_file],
    )
    run_joy_node = Node(
        package="virtual_joy",
        executable="virtual_joy",
    )
    joy_to_cmd_vel = Node(
        package="teleop_core",
        executable="joywithgui",
        parameters=["pioneer_base/teleop_core/config/joy-assign.yaml"],
    )
    ld.add_action(run_joy_node)
    ld.add_action(joy_to_cmd_vel)
    ld.add_action(run_cluster_node)


    return ld

