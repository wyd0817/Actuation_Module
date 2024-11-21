from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    num_namespaces = 2
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/bringup_4_toy_robots_launch.py'
            ]),
            launch_arguments={'namespace': f'c30r_{i}'}.items(),
        ) for i in range(num_namespaces)
    ])
