from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('pointcloud_preprocessing'), 'config', 'mapping.yaml'
    )

    return LaunchDescription([
        Node( #nav2_controller is used because nav2 does not currwently allow direct nav2_costmap_node execution
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[config_dir],
            remappings=[('cmd_vel', '/cmd_vel_unused')] # Remap cmd_vel to ensure no movement

        ),
    Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_local',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['controller_server']
            }]
        ),
    ])