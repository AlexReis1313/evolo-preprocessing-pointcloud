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
                         'target_frame': 'base_footprint',
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
                'print_time_metric':True,
                'save_time_metric': True,
                'base_link': 'base_link',
                'target_frame': 'base_footprint',
                'fixed_frame': 'odom',
                'cloud_frame': 'os_sensor',             
                'range_min': 1.0,
                'range_transition': 30.0, #15
                'range_max': 5000.0,
                'use_inf': True,
                'min_height_shortrange':-0.2, #-0.1,
                #for radial map output:
                'angle_increment_output_map': 0.2618, #pi/6 
                'angle_min_map': -1.570795,  # -M_PI/2
                'angle_max_map': 1.570795,  # M_PI/2
                # Radius Outlier Removal (ROR) filtering parameters
                'minimum_radius_paramB': 0.0, #0.03,#radius for neigbhours search for a point that is at a distance from the base_link = 0
                'minimum_radius_paramM': 0.04,#relationship of radisus of neighbours serach with distance of point to base_link
                'minimum_neighbours': 3,
                'filter_by_intensity': False,
                #Filtered output is given with time decay to help cluseting algorithm, here are the settings:
                'time_decay': 0.5,
                'TimeDecay_output_On_Fixed_Frame':False,
                #Ransac params
                'use_Ransac': True,
                'ransac_range_candidates': 15.0,
                'ransac_Maxheight_candidates':0.5,
                'ransac_Minheight_candidates':-1.5,
                'ransac_threshold_inliers': 0.2,
                'ransac_filter_height': 0.8
            }],
            name='pointcloud_preprocessing_filtering'
        )
    ])
