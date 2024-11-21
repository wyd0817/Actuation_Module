from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = [
        {
            'du_robot_name': 'du_123', 
            'c30r_robot_name': 'c30r_0',
            'static_transform_x': -1.0,
            'static_transform_y': 0.5,
            'static_transform_z': 0.0
        },
        {
            'du_robot_name': 'du_124', 
            'c30r_robot_name': 'c30r_1',
            'static_transform_x':-1.0,
            'static_transform_y': 0.0,
            'static_transform_z': 0.0
        }
    ]

    nodes = []
    
    for robot in robots:
        node = Node(
            package='operasim_nav2',
            executable='adapter_tf2_broadcaster_4_toy_robots',
            name=f"{robot['c30r_robot_name']}_node",
            namespace=robot['c30r_robot_name'],
            parameters=[{
                'du_robot_name': robot['du_robot_name'],
                'c30r_robot_name': robot['c30r_robot_name'],
                'static_transform_x': robot['static_transform_x'],
                'static_transform_y': robot['static_transform_y'],
                'static_transform_z': robot['static_transform_z']
            }],
            output='screen'
        )
        nodes.append(node)

    return LaunchDescription(nodes)
