from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([    
       
        Node(
            package='bb_dataass_tracking', executable='bounding_box_node_main',
            
            parameters=[{
                'use_sim_time': True,
                'fixed_frame': 'odom',
                'cloud_in': '/clustered_points',
                'R_cov': 0.1,
                'Q_cov': 0.5,
                'Q_bb_cov': 0.7 ,
                'min_velocity_threshold': 0.0,
                'newObjectThreshold': 15,
                'DD_cost_threshold': 15.0,
                'cov_upper_limit': 15.0,
                'prune_threshold': 40,
                'SaveBbMetric': False,
                'BbMetrics_file': 'bbTracking_realBuster2',
                'PrintTimeMetric': False,
                'SaveTimeMetric': False,
                'timingMetrics_file': 'BBTrack_timingFile.csv'


            }],
            name='object_tracking_node'
        )
    ])



