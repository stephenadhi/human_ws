# ROS 2 Social Locobot Development

This repository is a base for tests on the LoCoBot, both on the real robot and the simulation. The main repository of LoCoBot can be cloned from the repository [Interbotix ROS Rovers](https://github.com/Interbotix/interbotix_ros_rovers/tree/humble). 

## Prerequisites
You need to have the following installed on your machine:
- ROS 2 Humble
- Alternative: [Docker Desktop](https://www.docker.com/products/docker-desktop) for development using docker container


## Folder structure

    ├── 3rdparty
    ├── docker
    ├── nav2_soloco_controller
    ├── nav2_soloco_costmap_plugin
    ├── soloco_interfaces           
    ├── soloco_launch               
    ├── soloco_perception
    ├── LICENSE
    ├── README.md
    ├── .gitignore
    └── .gitmodules


## Installation
For first time installation, please refer to the [installation guide](Installation.md).

1. Pulling new changes locally
    ```
    git pull --recurse-submodules
    git submodule update --init --recursive
    ```
2. Applying new changes inside docker container
    ```
    # Run container
    sudo docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v /home/zedbox/ros2_ws:/home/zedbox/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0
    # Pull new changes
    cd home/zedbox/ros2_ws/src/human_ws
    git pull
    cd home/zedbox/ros2_ws
    colcon build --packages-select soloco_perception nav2_soloco_controller soloco_launch soloco_interfaces
    ```

## Custom Launch Examples
We provide general launch files for perception and navigation. Our modified version of the LoCoBot have two computing device onboard: Intel NUC and ZED Box (Jetson Xavier NX).

First, SSH to the Intel NUC computer
    
    ssh locobot@locobot
    
Optional: choose unique domain ID to prevent interference from other running ROS 2 system

    export ROS_DOMAIN_ID=2 # Adjust to your settings, default=0

Go to the directory and source installation

    cd ~/interbotix_ws
    source install/setup.bash && source /opt/ros/humble/setup.bash
    
### Default LoCoBot launch with navigation2 stack + SLAM, and visualization:

    ros2 launch soloco_launch intelnuc_locobot_bringup.launch.py nav2_param_filename:=smac_mppi_nav2_params.yaml

### LoCoBot launch with custom neural motion planner as controller:

    ros2 launch soloco_launch intelnuc_locobot_bringup.launch.py use_nav2_slam:=True nav2_param_filename:=smac_soloco_nav2_params.yaml use_soloco_controller:=True

### For launching ZED human perception and multi-tracking module:
SSH to the ZED Box

    ssh zedbox@10.42.0.230
    
Run the shell script to bringup our ROS modules inside a docker container

    ./docker_zedbox_bringup.sh

### Visualization in remote PC:
Source installation and set the same ROS domain ID.

    export ROS_DOMAIN_ID=2 # Adjust to your settings, default=0
    source install/setup.bash

Launch RViZ:

    ros2 launch soloco_launch remote_view.launch.py

## Simulation Example with Pedsim
Launch Locobot in Gazebo, optionally with GUI, pedestrian simulator, and RViZ. Odometry is based on ground truth with gaussian noise=0.001. The default launch is the following command:

    ros2 launch soloco_launch locobot_sim.launch.py use_gazebo_gui:=false use_pedsim:=true use_soloco_controller:=true launch_remote_view:=true

## Rosbag Example
To record only robot data, run the following command
    ros2 bag record /locobot/commands/velocity /locobot/odom /map /local_costmap/costmap /global_costmap/costmap /tf /tf_static /plan /visualization/predicted_future /human/interpolated_history /robot/ego_trajectory /goal_pose /locobot/robot_description /zed2/zed_node/left_raw/image_raw_color

To record data for visualization purposes, run the following command

    ros2 bag record /locobot/commands/velocity /locobot/odom /map /local_costmap/costmap /global_costmap/costmap /tf /tf_static /plan /visualization/predicted_future /visualization/human_tracks /visualization/robot_track /visualization/subgoal /visualization/human_bounding_boxes /goal_pose /locobot/robot_description /zed2/zed_node/left_raw/image_raw_color

To record simulation data, run the following command
    ros2 bag record /locobot/diffdrive_controller/cmd_vel_unstamped /locobot/odom /map /local_costmap/costmap /global_costmap/costmap /tf /tf_static /plan /human/simulated_agents /human/predicted_future /human/interpolated_history /robot/ego_trajectory /goal_pose /locobot/robot_description