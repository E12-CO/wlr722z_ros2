import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'wlr722z_ros2'
    
    param_dir = launch.substitutions.LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'params',
            'lidar.yaml'))
    
    wlr722z_instant = launch_ros.actions.Node(
        package=pkg_name,
        executable='wlr_node',
        output='screen',
        parameters=[param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'wlr722z_param_dir',
            default_value=param_dir,
            description='Full path to main parameter file to load'),
        wlr722z_instant,
    ])
