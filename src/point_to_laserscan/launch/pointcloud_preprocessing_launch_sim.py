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
            parameters=[{'use_sim_time': False,
                         'base_link': 'Evolo/base_link_gt',
                         'fixed_frame': 'map_gt',
                         'target_frame': 'base_footprint',
                         }],
            name='base_footprint_publisher_node'
        ),
        

        Node(
            package='pointcloud_preprocessing', executable='pointcloud_preprocessing_node',
            remappings=[('cloud_in', 'Evolo/Lidar/MidRes')],
            parameters=[{
                'use_sim_time':False,
                'simulation_mode':True,
                'target_frame': 'base_footprint',
                'fixed_frame': 'map_gt',
                'cloud_frame': 'Evolo/lidar_link_gt',      
                'base_link': 'Evolo/base_link_gt',
                'transform_tolerance': 0.6,
                'min_height_longrange': -1.0, #-8 for real evolo
                'max_height_longrange': 8.0,
                'range_min': 1.5,
                'range_max': 5000.0,
                'use_inf': True,
                'min_height_shortrange': -0.2,
                'max_height_shortrange': 6.0,
                'range_transition': 20.0, #pi/6
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
                'use_Ransac': False,
                'ransac_range_candidates': 15.0,
                'ransac_Maxheight_candidates':0.5,
                'ransac_Minheight_candidates':-1.5,
                'ransac_threshold_inliers': 0.2,
                'ransac_filter_height': 0.5

            }],
            name='pointcloud_preprocessing_filtering'
        )
    ])
