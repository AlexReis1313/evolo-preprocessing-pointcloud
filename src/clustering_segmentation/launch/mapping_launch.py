
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
                {"clustering": False},
                {"DynamicStatic_clusters_segmentation": False},
                {"static_mapping": False}, #by default, there is a mapping that assumes the same baysian filter as the one defined for static map, when DynamicStatic_clusters_segmentation is false. It will use the filtered/ls/laserscan topic instead of static/pointcloud topic
                {"PrintTimeMetric": False},
                {"SaveTimeMetric": False}
            ]
        )
    ])