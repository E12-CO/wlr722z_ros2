import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'wlr722z_ros2'
    
    hinson_de4511_instant = launch_ros.actions.Node(
        package=pkg_name,
        executable='wlr_node',
        output='screen',
        parameters=[param_dir]
    )
    
    return launch.LaunchDescription([
        hinson_de4511_instant,
    ])
