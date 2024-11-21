#!/bin/bash

# Source ROS2 setup script
source install/local_setup.bash

# Loop through namespaces and launch each robot with corresponding rviz configuration
# for i in 0 2 5; do
for i in {0..5}; do
    index=$(printf "%02d" $((i + 1)))  # 0 -> 01, 1 -> 02, 2 -> 03, 3 -> 04, 4 -> 05, 5 -> 06
    rviz_config_file="/root/share/ros2_ws/rviz_configures/rviz2_robot_dump_truck_$index.rviz"
    ros2 launch operasim_nav2 rviz_launch.py namespace:=c30r_$i rviz_config:=$rviz_config_file &
done

for i in {0..1}; do
    index=$(printf "%02d" $((i + 1))) # 0 -> 01, 1 -> 02
    rviz_config_file="/root/share/ros2_ws/rviz_configures/rviz2_robot_excavator_$index.rviz"
    ros2 launch operasim_nav2 rviz_launch.py namespace:=zx120_$i rviz_config:=$rviz_config_file &
done

wait
