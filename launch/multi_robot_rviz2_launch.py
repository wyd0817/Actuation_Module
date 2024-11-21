from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # List to store all robot RViz launch descriptions
    robot_rviz_launch_descriptions = []

    # Loop through namespaces for dump trucks
    for i in range(6):
        namespace = f'c30r_{i}'
        index = f'{i+1:02d}'  # Convert to two-digit index
        rviz_config_file = f'/root/share/ros2_ws/rviz_configures/rviz2_robot_dump_truck_{index}.rviz'
        robot_rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/rviz_launch.py'
            ]),
            launch_arguments={
                'namespace': namespace,
                'rviz_config': rviz_config_file
            }.items(),
        )
        robot_rviz_launch_descriptions.append(robot_rviz_launch)

    # Loop through namespaces for excavators
    for i in range(2):
        namespace = f'zx120_{i}'
        index = f'{i+1:02d}'  # Convert to two-digit index
        rviz_config_file = f'/root/share/ros2_ws/rviz_configures/rviz2_robot_excavator_{index}.rviz'
        robot_rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/rviz_launch.py'
            ]),
            launch_arguments={
                'namespace': namespace,
                'rviz_config': rviz_config_file
            }.items(),
        )
        robot_rviz_launch_descriptions.append(robot_rviz_launch)

    return LaunchDescription(robot_rviz_launch_descriptions)
