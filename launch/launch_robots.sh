#!/bin/bash

# Source ROS2 setup script
source install/local_setup.bash

# Loop through namespaces and launch each robot
for i in {0..5}; do
    ros2 launch operasim_nav2 bringup_launch.py namespace:=c30r_$i &
done

for i in {0..1}; do
    ros2 launch operasim_nav2 bringup_launch.py namespace:=zx120_$i &
done

wait
