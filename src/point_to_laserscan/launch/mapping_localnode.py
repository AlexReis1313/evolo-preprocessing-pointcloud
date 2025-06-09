from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('pointcloud_to_laserscan'), 'config', 'Local_mapping.yaml'
    )
    return LaunchDescription([
        # Lifecycle Manager to manage lifecycle nodes
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_local',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['local_costmap']
            }]
        ),

        # Costmap node
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='local_costmap',
            output='screen',
            parameters=[config_dir],

            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
    ])