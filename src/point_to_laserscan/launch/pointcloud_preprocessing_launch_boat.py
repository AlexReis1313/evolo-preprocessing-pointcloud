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
            package='pointcloud_preprocessing', executable='base_footprint_publisher',
            parameters=[{'use_sim_time': True,
                         'base_link': 'base_link',
                         'target_frame': 'base_footprint_2',
                         'fixed_frame': 'odom',
                         'zero_heigh_footprint':False #If false, will have same height as base_link

                         }],
            name='base_footprint_publisher_node'
        ),
        

        Node(
            package='pointcloud_preprocessing', executable='pointcloud_preprocessing_node',
            remappings=[('cloud_in', 'ouster/points')],

            parameters=[{
                'use_sim_time': True,  # Enable simulation time
                'base_link': 'base_link',
                'target_frame': 'base_footprint_2',
                'fixed_frame': 'odom',
                'cloud_frame': 'os_sensor',             
                'transform_tolerance': 0.01,
                'min_height_longrange': -8.0,
                'max_height_longrange': 2.0,
                'angle_min_laserscan':  -3.14159,  # -M_PI
                'angle_max_laserscan': 3.14159,  # M_PI
                'angle_increment': 0.003067,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 1.0,
                'range_max': 5000.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'min_height_shortrange': -0.6,
                'max_height_shortrange': 2.0,
                'range_transition': 15.0,
                #for radial map output:
                'angle_increment_output_map': 0.2618, #pi/6 
                'angle_min_map': -1.570795,  # -M_PI/2
                'angle_max_map': 1.570795,  # M_PI/2
                # Radius Outlier Removal (ROR) filtering parameters
                'minimum_radius_paramB': 0.01,#radius for neigbhours search for a point that is at a distance from the base_link = 0
                'minimum_radius_paramM': 0.05, #0.016,#relationship of radisus of neighbours serach with distance of point to base_link
                'minimum_neighbours': 3,
                'filter_by_intensity': False,
                'use_Ransac': False,


            }],
            name='pointcloud_preprocessing_filtering'
        )
    ])
