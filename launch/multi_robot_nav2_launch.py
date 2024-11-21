from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # List to store all robot launch descriptions
    robot_launch_descriptions = []

    # Loop through namespaces for dump trucks
    for i in range(6):
        namespace = f'c30r_{i}'
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/bringup_launch.py'
            ]),
            launch_arguments={'namespace': namespace}.items(),
        )
        robot_launch_descriptions.append(robot_launch)

    # Loop through namespaces for excavators
    for i in range(2):
        namespace = f'zx120_{i}'
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/bringup_launch.py'
            ]),
            launch_arguments={'namespace': namespace}.items(),
        )
        robot_launch_descriptions.append(robot_launch)

    return LaunchDescription(robot_launch_descriptions)

