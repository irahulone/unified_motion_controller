import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def check_service_response(context, *args, **kwargs):
    service_checker = Node(
        package='register_service',
        executable='register_service_req',
        name='service_checker',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'theta': LaunchConfiguration('t')
        }]  
    )

    return [service_checker]

def launch_setup(context, *args, **kwargs):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    xacro_file = os.path.join(pkg_share, 'src/description/pioneer_robot.xacro')
    robot_id = LaunchConfiguration("robot_id").perform(context)

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
                    " x:=", 
                    LaunchConfiguration("x"),
                    " y:=", 
                    LaunchConfiguration("y"),
                    " t:=", 
                    LaunchConfiguration("t")
                ]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_prefix": f"{robot_id}/",
            }]
        ),
        Node(
            package='register_service',
            executable='remove_service_req',
            name='robot_remove_request',
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id')
            }]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}/world"],
        ),
    ]
    return main_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="p1", description="Robot ID"),
        DeclareLaunchArgument("x", default_value="0.0", description="X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Y position"),
        DeclareLaunchArgument("t", default_value="0.0", description="Theta"),
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        OpaqueFunction(function=check_service_response),
        OpaqueFunction(function=launch_setup),
    ])
