from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the first launch file (multi_robot_launch.py)
    multi_adapter_4_toy_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('operasim_nav2'), '/launch/multi_adapter_4_toy_robots_launch.py'
        ]),
    )

    # Include the second launch file (multi_bringup.launch.py)
    robot_swarm_4_toy_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('operasim_nav2'), '/launch/robot_swarm_4_toy_robots_launch.py'
        ]),
    )

    # Include the third launch file (multi_rviz_launch.py)
    robot_swarm_rviz_4_toy_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('operasim_nav2'), '/launch/robot_swarm_rviz_4_toy_robots_launch.py'
        ]),
    )

    return LaunchDescription([
        # Start multi_robot_launch.py immediately
        multi_adapter_4_toy_robots_launch,
        # Wait for 5 seconds before starting the other two launch files
        TimerAction(
            period=5.0,
            actions=[robot_swarm_4_toy_robots_launch, robot_swarm_rviz_4_toy_robots_launch]
        ),
    ])
