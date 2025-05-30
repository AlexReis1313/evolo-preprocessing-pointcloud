from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
       Node(
            package='pointcloud_to_laserscan', executable='base_footprint_publisher',
            parameters=[{'use_sim_time': True}],
            name='base_footprint_publisher_node'
        ),
       
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/Evolo/Lidar/HighRes'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan/filtered'])],
            parameters=[{
                'use_sim_time':False,
                'target_frame': 'base_footprint',
                'fixed_frame': 'map_gt',
                'cloud_frame': 'Evolo/lidar_link_gt',      
                'base_link': 'Evolo/base_link_gt',
                'transform_tolerance': 0.01,
                'min_height_longrange': -1.0, #-8 for real evolo
                'max_height_longrange': 8.0,
                'angle_min_laserscan': -3.14159,  # -M_PI
                'angle_max_laserscan': 3.14159,  # M_PI
                'angle_increment': 0.003067,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 1.5,
                'range_max': 5000.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'min_height_shortrange': -0.2,
                'max_height_shortrange': 6.0,
                'range_transition': 20.0, #pi/6
                'angle_increment_output_map': 0.2618, #pi/6 
                'angle_min_map': -1.570795,  # -M_PI/2
                'angle_max_map': 1.570795,  # M_PI/2
                'minimum_radius_paramB': 0.05,#radius for neigbhours search for a point that is at a distance from the base_link = 0
                'minimum_radius_paramM': 0.0125,#relationship of radisus of neighbours serach with distance of point to base_link
                'minimum_neighbours': 2,
                'filter_by_intensity': False

            }],
            name='pointcloud_preprocessing_filtering'
        )
    ])
