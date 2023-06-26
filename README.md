# ROS 2 Social Locobot Development

This repository is a base for tests on the LoCoBot, both on the real robot and the simulation. The main repository of LoCoBot can be cloned from the repository [Interbotix ROS Rovers](https://github.com/Interbotix/interbotix_ros_rovers/tree/humble). 

## Prerequisites
You need to have the following installed on your machine:
- ROS 2 Humble
- Alternative: [Docker Desktop](https://www.docker.com/products/docker-desktop) for development using docker container


## Folder structure

    ├── 3rdparty
    ├── docker
    ├── nav2_soloco_costmap_plugin
    ├── soloco_interfaces           
    ├── soloco_launch               
    ├── soloco_perception
    ├── nav2_soloco_controller
    ├── LICENSE
    ├── README.md
    ├── .gitignore
    └── .gitmodules


## Installation
1. Make a ROS 2 workspace folder and clone this repository inside src/ folder
    ```
    mkdir -p workspaces/humble/src
    git clone --recursive https://git.uni-due.de/locobot/human_ws.git
    # Build Navigation2 from source (optional, necessary for using MPPI controller)
    git clone -b humble https://github.com/ros-planning/navigation2.git
    ```

2. Pulling new changes locally
    ```
    git pull --recurse-submodules
    git submodule update --init --recursive
    ```
3. Applying new changes inside docker container
    ```
    # Run container
    sudo docker run --runtime nvidia -it --rm --network host --privileged -v /dev:/dev -v /home/zedbox/ros2_ws:/home/zedbox/ros2_ws stephenadhi/ros2:humble-l4t-r35.2-zedsdk-4.0
    # Pull new changes
    cd home/zedbox/ros2_ws/src/human_ws
    git pull
    cd home/zedbox/ros2_ws
    colcon build --packages-select soloco_perception nav2_soloco_controller soloco_launch soloco_interfaces
    ```

4. Install ROS dependencies
    ```
    cd workspaces/humble/
    rosdep install -i -y -r --from-paths src --rosdistro humble
    ```

5. Build everything inside the workspace
   ```
    colcon build --packages-select zed_interfaces
    source install/setup.bash
    colcon build --packages-select soloco_interfaces
    colcon build
   ```

6. Define robot base type and model
   ```
   export INTERBOTIX_XSLOCOBOT_BASE_TYPE=kobuki
   export INTERBOTIX_XSLOCOBOT_ROBOT_MODEL=locobot_base
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

    ros2 launch soloco_launch intelnuc_locobot_bringup.launch.py nav2_param_filename:=smac_mppi_nav2_params.yaml use_soloco_controller:=True

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

## Simulation Example
Launch Locobot in Gazebo with GUI and optionally pedestrian simulator

    ros2 launch soloco_launch locobot_sim.launch.py use_gazebo_gui:=true use_pedsim:=false

