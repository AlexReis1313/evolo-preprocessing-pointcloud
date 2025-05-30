from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare arguments only once
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("slam_toolbox"),
            'config', 'mapper_params_online_async.yaml'),
        description='Path to the SLAM params file')

    # First instance of SLAM Toolbox
    start_slam_toolbox_1 = Node(
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='mappingNode1',
        output='screen'
    )

    # Second instance of SLAM Toolbox
    start_slam_toolbox_2 = Node(
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='mappingNode2',
        output='screen'
    )

    # Include launch from pointcloud_to_laserscan
    pcl_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pointcloud_to_laserscan'),
                'launch',
                'sample_pointcloud_to_laserscan_launch.py'
            )
        )
    )

    # Other nodes
    ufo_mapping_node = Node(
        package='ufo_mapping',
        executable='server_2d',
        name='ufo_mapping_server',
        output='screen'
    )

    bounding_box_node = Node(
        package='bb_dataass_tracking',
        executable='bounding_box_node_main',
        name='bounding_box_tracker',
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_slam_params_file_cmd,
        start_slam_toolbox_1,
        start_slam_toolbox_2,
        pcl_to_laserscan_launch,
        ufo_mapping_node,
        bounding_box_node
    ])
