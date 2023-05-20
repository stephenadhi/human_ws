#!/bin/bash

docker run --runtime nvidia -it --rm --network host --privileged \
-v /dev:/dev \
-v /home/zedbox/ros2_ws:/home/zedbox/ros2_ws \
stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0 \
bash -c "cd /home/zedbox/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=2 && ros2 launch soloco_launch zedbox_bringup.launch.py"
