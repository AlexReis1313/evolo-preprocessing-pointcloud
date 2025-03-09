from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""    
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'ouster/points_corrected'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan2/short'])],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 6.0,
                'min_height_shortrange': -15.0,
                'max_height_shortrange': 15.0,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.003067,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 1.5,
                'range_transition': 5.0,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_short'
        ),
        """
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),

        

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'ouster/points_corrected'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan2/long'])],
            parameters=[{
                'target_frame': 'os_sensor',
                'transform_tolerance': 0.01,
                'min_height': -10.0,
                'max_height': 10.0,
                'min_height_shortrange': -15.0,
                'max_height_shortrange': 15.0,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.003067,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 20.0,
                'range_transition': 5.0,

                'range_max': 300.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_long'
        )
    ])
