# ROS 2 Social Locobot Development

This repository is a base for tests on the LoCoBot, both on the real robot and the simulation. The main repository of LoCoBot can be cloned from the repository [Interbotix ROS Rovers](https://github.com/Interbotix/interbotix_ros_rovers/tree/humble). 

## Prerequisites
You need to have the following installed on your machine:
- ROS 2 Humble
- Alternative: [Docker Desktop](https://www.docker.com/products/docker-desktop) for development using docker container


## Folder structure


    ├── nav2_soloco_costmap_plugin
    ├── soloco_interfaces           
    ├── soloco_launch               
    ├── soloco_perception
    ├── soloco_planner
    ├── LICENSE
    ├── README.md
    ├── .gitignore
    └── .gitmodules


## Installation
1. Make a ROS 2 workspace folder and clone this repository inside src/ folder
    ```
    mkdir -p workspaces/humble/src
    git clone --recursive https://git.uni-due.de/locobot/human_ws.git
    ```
2. Install ROS dependencies
    ```
    cd workspaces/humble/
    rosdep install -i -y -r --from-paths src --rosdistro humble
    ```
3. Build everything inside the workspace
   ```
    colcon build --packages-select zed_interfaces
    . install/setup.bash
    colcon build --packages-select soloco_interfaces
    colcon build
   ```

## Launching Examples
We provide general launch files for perception and navigation. Our modified version of the LoCoBot have two computing device onboard: Intel NUC and ZED Box (Jetson Xavier NX).

For launching LoCoBot, navigation2 stack + SLAM, and visualization:
   ```
    ros2 launch intelnuc_bringup.launch.py
   ```
For launching LoCoBot, nav2_planner_server + SLAM, neural planner, human tf publisher, and visualization:
   ```
    # SSH to the Intel NUC computer
    cd ~/interbotix_ws
    . install/setup.bash && . /opt/ros/humble/setup.bash
    ros2 launch intelnuc_bringup.launch.py launch_neural_planner:=True run_human_tf:=True
   ```
For launching ZED human perception and multi-tracking module:
   ```
    # SSH to the ZED Box 
    ./docker_zedbox_bringup.sh
   ```
<!-- For launching visualization in remote PC:
    ```
    source install/setup.bash
    ``` -->
