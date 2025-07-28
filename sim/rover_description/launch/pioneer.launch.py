import os
import launch
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

COLORS = {
    'p1': [1.0, 0.0, 0.0],
    'p2': [0.0, 1.0, 0.0],
    'p3': [0.0, 0.0, 1.0],
    'p4': [1.0, 1.0, 0.0],
    'p5': [1.0, 0.0, 1.0],
    'p6': [0.0, 1.0, 1.0],
}

def launch_setup(context, *args, **kwargs):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    xacro_file = os.path.join(pkg_share, f'src/description/pioneer_robot.xacro')
    robot_id = LaunchConfiguration("robot_id").perform(context)
    hw = LaunchConfiguration("hw").perform(context)
    desired = LaunchConfiguration("desired").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    t = float(LaunchConfiguration("t").perform(context))


    main_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="state_publisher",
            namespace=robot_id,
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", 
                    xacro_file, 
                    f" r:={COLORS[robot_id][0]} g:={COLORS[robot_id][1]} b:={COLORS[robot_id][2]} a:=1.0"
                ]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_prefix": f"{robot_id}/",
            }]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}/world"],
        ),
        Node(
            package='fake_rover_state_controller',
            executable='rover_sim',
            name='rover_sim',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'x': x,
                'y': y,
                't': t,
            }]
        ),
    ]
    if hw == 'hw':
        main_nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                namespace=f"{robot_id}hw",
                output="screen",
                parameters=[{
                    "robot_description": Command([
                        "xacro ", 
                        xacro_file, 
                        f" r:={COLORS[robot_id][0]} g:={COLORS[robot_id][1]} b:={COLORS[robot_id][2]} a:=",
                        LaunchConfiguration("hwa")                    
                    ]),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_prefix": f"{robot_id}hw/",
                }]
            )
        )
        main_nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}hw/world"],
            ),
        )
    if desired == 'desired':
        main_nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                namespace=f"{robot_id}desired",
                output="screen",
                parameters=[{
                    "robot_description": Command([
                        "xacro ", 
                        xacro_file, 
                        f" r:={COLORS[robot_id][0]} g:={COLORS[robot_id][1]} b:={COLORS[robot_id][2]} a:=",
                        LaunchConfiguration("a")                    
                    ]),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_prefix": f"{robot_id}desired/",
                }]
            )
        )
        main_nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}desired/world"],
            ),
        )
    return main_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="p1", description="Robot ID"),
        DeclareLaunchArgument("x", default_value="0.0", description="X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Y position"),
        DeclareLaunchArgument("t", default_value="0.0", description="Theta"),
        DeclareLaunchArgument("a", default_value="1.0", description="Transparency"),
        DeclareLaunchArgument("hwa", default_value="0.5", description="Transparency of hw"),
        DeclareLaunchArgument("hw", default_value="", description="Transparency"),
        DeclareLaunchArgument("desired", default_value="", description="Transparency"),
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        OpaqueFunction(function=launch_setup),
    ])
