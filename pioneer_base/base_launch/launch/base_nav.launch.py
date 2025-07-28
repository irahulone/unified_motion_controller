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
    
    # Nodes
    auto_nav_p2 = Node(
        package="auto_nav_core",
        executable="auto_nav",
        name="auto_nav_p2",
        parameters=[{"robot_id": "p2"}]
    )
    auto_nav_p3 = Node(
        package="auto_nav_core",
        executable="auto_nav",
        name="auto_nav_p3",
        parameters=[{"robot_id": "p3"}]
    )
    auto_nav_p4 = Node(
        package="auto_nav_core",
        executable="auto_nav",
        name="auto_nav_p4",
        parameters=[{"robot_id": "p4"}]
    )

    nav_controller = Node(
        package="auto_nav_core",
        executable="nav_controller",
        name="nav_controller",
        parameters=['src/auto_nav_core/config/navparams.yaml']
    )

    mode_controller = Node(
        package="mode_select",
        executable="mode_select_multi",
        name="mode_controller",
        parameters=['src/mode_select/config/mode_sel.yaml']
    )
    
    # Joy
    run_joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    joy_to_cmd_vel = Node(
        package="teleop_core",
        executable="joy2cmd",
    )

    demux = Node(
        package="teleop_core",
        executable="cmd_demux",
        parameters=['src/teleop_core/config/demux.yaml']
    )
    

    ld.add_action(auto_nav_p2)
    ld.add_action(auto_nav_p3)
    ld.add_action(auto_nav_p4)
    ld.add_action(nav_controller)
    ld.add_action(mode_controller)
    ld.add_action(run_joy_node)
    ld.add_action(joy_to_cmd_vel)
    ld.add_action(demux)

    return ld


