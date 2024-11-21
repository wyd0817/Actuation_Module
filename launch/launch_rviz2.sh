#!/bin/bash

# Source ROS2 setup script
source install/local_setup.bash

# Loop through namespaces and launch each robot with corresponding rviz configuration
# for i in 0 2 5; do
for i in {0..5}; do
    ros2 launch operasim_nav2 rviz_launch.py namespace:=c30r_$i &
done

for i in {0..1}; do
    ros2 launch operasim_nav2 rviz_launch.py namespace:=zx120_$i &
done

wait
