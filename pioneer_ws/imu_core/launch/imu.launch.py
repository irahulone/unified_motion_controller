import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    ld=launch.LaunchDescription()
    config=os.path.join(
        get_package_share_directory('imu_core'),
        'config',
        'paramsIMU.yaml'
    )
    node=launch_ros.actions.Node(
        package='imu_core',
        name = 'robot1_imu',
        executable='run_imu',
        parameters=[config]
    )
    ld.add_action(node)
    return ld