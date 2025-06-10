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
            parameters=[{'use_sim_time': True,
                         'base_link': 'base_link',
                         'target_frame': 'base_footprint',
                         'fixed_frame': 'odom',
                         'zero_heigh_footprint':False

                         }],
            name='base_footprint_publisher_node'
        ),
        

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'ouster/points'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan/filtered'])],

            parameters=[{
                'use_sim_time': True,  # Enable simulation time
                'base_link': 'base_link',
                'target_frame': 'base_footprint',
                'fixed_frame': 'odom',
                'cloud_frame': 'os_sensor',             
                'transform_tolerance': 0.01,
                'min_height_longrange': -6.0,
                'max_height_longrange': 6.0,
                'angle_min_laserscan': -3.14159,  # -M_PI
                'angle_max_laserscan': 3.14159,  # M_PI
                'angle_increment': 0.003067,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 1.0,
                'range_max': 5000.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'min_height_shortrange':-0.2, #-0.1,
                'max_height_shortrange': 4.0,
                'range_transition': 30.0, #15
                #for radial map output:
                'angle_increment_output_map': 0.2618, #pi/6 
                'angle_min_map': -1.570795,  # -M_PI/2
                'angle_max_map': 1.570795,  # M_PI/2
                # Radius Outlier Removal (ROR) filtering parameters
                'minimum_radius_paramB': 0.03, #0.03,#radius for neigbhours search for a point that is at a distance from the base_link = 0
                'minimum_radius_paramM': 0.02,#relationship of radisus of neighbours serach with distance of point to base_link
                'minimum_neighbours': 2 ,
                'filter_by_intensity': False,
                #Filtered output is given with time decay to help cluseting algorithm, here are the settings:
                'time_decay': 2.5,
                'TimeDecay_output_On_Fixed_Frame':False,
                #Ransac params
                'ransac_range_candidates': 15.0,
                'ransac_Maxheight_candidates':0.5,
                'ransac_Minheight_candidates':-1.5,
                'use_Ransac': True,
                'ransac_threshold_inliers': 0.2,
                'ransac_filter_height': 0.5
            }],
            name='pointcloud_preprocessing_filtering'
        )
    ])
