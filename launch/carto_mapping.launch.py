# Launch file for mapping with WLR-722Z 3D LiDAR
# Coded by TinLethax
import os

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions
import xacro

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

     # Specify the name of the package and path to xacro file within the package
    pkg_name = 'wlr722z_ros2'
    file_subpath = 'description/wlr722z.urdf'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Configure node launch information 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.01')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory', default= os.path.join(get_package_share_directory(pkg_name), 'params') )
    # Configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', default='wlr722z_mapping.lua')
    
    # Configure the node
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # LiDAR driver
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
    
    # Cartographer SLAM
    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        remappings=[
                ('imu', 'imu'),
                ('points2', 'cloud')
                ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                  ]
    )

    cartographer_occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'wlr722z_param_dir',
            default_value=param_dir,
            description='Full path to main parameter file to load'),
        wlr722z_instant,
        node_robot_state_publisher,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
