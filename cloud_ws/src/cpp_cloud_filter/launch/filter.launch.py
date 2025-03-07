from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    filter = Node( package='cpp_cloud_filter',
                    executable='filter',
                    name=f'cpp_filter',
                    remappings=[
                        ('~/input', '/vehicle/velodyne/point_cloud')
                    ])

    return LaunchDescription([
        filter
    ])
