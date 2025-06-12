from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([    
       
        Node(
            package='bb_dataass_tracking', executable='bounding_box_node',
            
            parameters=[{
                'use_sim_time': True,  # Enable simulation time
            }],
            name='bouding_box_pkg'
        )
    ])
