import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # List machine ids
    machine_ids = LaunchConfiguration("machine_ids")
    declare_machine_ids_cmd = DeclareLaunchArgument(
        "machine_ids",
        default_value=["c30r_0", "c30r_1"],
        description="Machine ids to be used to send tf",
    )
    # Sim time
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    node = Node(
        package="operasim_nav2",
        executable="odom_pose_to_tf2_broadcaster",
        parameters=[{"use_sim_time": use_sim_time, "machine_ids": machine_ids}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_machine_ids_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(node)

    return ld
