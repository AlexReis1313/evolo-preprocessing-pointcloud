from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    custom_config_dir = os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'config')

    return LaunchDescription([
        Node(
            package= 'nav2_map_server', #'nav2_costmap_2d',
            executable= 'map_server',#'costmap_2d_node',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True,
                 'yaml_filename': os.path.join(custom_config_dir, 'mapping.yaml')
                 },
                
            ],
            #remappings=[
            #    ('/scan', '/your_pointcloud_converted_to_scan_or_obstacles'),
            #    ('/tf', '/tf'),
            #    ('/tf_static', '/tf_static')
            #]
        ),
        # Lifecycle Manager to automatically activate map
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
    ])
