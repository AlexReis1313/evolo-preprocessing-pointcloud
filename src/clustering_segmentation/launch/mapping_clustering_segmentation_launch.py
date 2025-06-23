

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clustering_segmentation',  # Replace with your actual package name
            executable='clustering_segmentation',
            name='occupancy_grid',
            output='screen',
            parameters=[
                {"clustering": True},
                {"DynamicStatic_clusters_segmentation": True},
                {"static_mapping": True},
                {"static_points_topic": "static/laserscan"},
                {"clustering_points_topic_in": "filtered/ls/pointcloud/accumulated"},
                {"clustering_points_topic_out": "clustered_points"},
                {"PrintTimeMetric": False},
                {"SaveTimeMetric": False}
            ]
        )
    ])