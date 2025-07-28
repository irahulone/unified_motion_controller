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
    run_joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    joy_to_cmd_vel = Node(
        package="teleop_core",
        executable="joy2cmd",
        parameters=["pioneer_base/teleop_core/config/joy-heading.yaml"],
    )

    demux = Node(
        package="teleop_core",
        executable="cmd_demux",
        parameters=["pioneer_base/teleop_core/config/demux-heading.yaml"],
    )
    
    heading_controller_p2 = Node(
        package="controller",
        executable="heading_controller",
        parameters=[{"rover_id": "p2"}]
    )

    heading_controller_p3 = Node(
        package="controller",
        executable="heading_controller",
        parameters=[{"rover_id": "p3"}]
    )

    heading_controller_p4 = Node(
        package="controller",
        executable="heading_controller",
        parameters=[{"rover_id": "p4"}]
    )

    ld.add_action(run_joy_node)
    ld.add_action(joy_to_cmd_vel)
    ld.add_action(demux)
    ld.add_action(heading_controller_p2)
    ld.add_action(heading_controller_p3)
    ld.add_action(heading_controller_p4)

    return ld


