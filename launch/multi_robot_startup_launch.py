from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    machine_ids = LaunchConfiguration("machine_ids")
    declare_machine_ids_cmd = DeclareLaunchArgument(
        "machine_ids",
        default_value="[c30r_0, c30r_1, c30r_2, c30r_3, c30r_4, c30r_5, zx120_0, zx120_1]",
        description="Machine ids to be used to send tf",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    # Node for odom_pose_to_tf2_broadcaster
    odom_pose_to_tf2_node = Node(
        package="operasim_nav2",
        executable="odom_pose_to_tf2_broadcaster",
        parameters=[{"use_sim_time": use_sim_time, "machine_ids": machine_ids}],
    )

    # List to store all robot bringup launch descriptions
    robot_bringup_launch_descriptions = []

    # Loop through namespaces for dump trucks
    for i in range(6):
        namespace = f'c30r_{i}'
        robot_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/bringup_launch.py'
            ]),
            launch_arguments={'namespace': namespace}.items(),
        )
        robot_bringup_launch_descriptions.append(robot_bringup_launch)

    # Loop through namespaces for excavators
    for i in range(2):
        namespace = f'zx120_{i}'
        robot_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('operasim_nav2'), '/launch/bringup_launch.py'
            ]),
            launch_arguments={'namespace': namespace}.items(),
        )
        robot_bringup_launch_descriptions.append(robot_bringup_launch)

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

    # Combine all actions into one LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_machine_ids_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(odom_pose_to_tf2_node)
    
    for action in robot_bringup_launch_descriptions:
        ld.add_action(action)
    
    for action in robot_rviz_launch_descriptions:
        ld.add_action(action)

    return ld
