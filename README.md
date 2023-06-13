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
    # Build Navigation2 from source (optional, necessary for using MPPI controller)
    git clone -b humble https://github.com/ros-planning/navigation2.git
    ```

2. Pulling new changes
    ```
    git pull --recurse-submodules
    git submodule update --init --recursive
    ```

3. Install ROS dependencies
    ```
    cd workspaces/humble/
    rosdep install -i -y -r --from-paths src --rosdistro humble
    ```

4. Build everything inside the workspace
   ```
    colcon build --packages-select zed_interfaces
    source install/setup.bash
    colcon build --packages-select soloco_interfaces
    colcon build
   ```

5. Define robot base type and model
   ```
   export INTERBOTIX_XSLOCOBOT_BASE_TYPE=kobuki
   export INTERBOTIX_XSLOCOBOT_ROBOT_MODEL=locobot_base
   ```

## Launching Examples
We provide general launch files for perception and navigation. Our modified version of the LoCoBot have two computing device onboard: Intel NUC and ZED Box (Jetson Xavier NX).

Choose unique domain ID to prevent interference from other running ROS 2 system
    ```
    export ROS_DOMAIN_ID=2 # Adjust to your settings, default=0
    ```
For launching LoCoBot, navigation2 stack + SLAM, and visualization:
   ```
    ros2 launch intelnuc_bringup.launch.py
   ```
Default LoCoBot with Nav2 + SLAM:
   ```
    # SSH to the Intel NUC computer
    cd ~/interbotix_ws
    source install/setup.bash && source /opt/ros/humble/setup.bash
    ros2 launch soloco_launch intelnuc_locobot_nav2_bringup.launch.py nav2_param_filename:=smac_mppi_nav2_params.yaml
   ```
For launching ZED human perception and multi-tracking module:
   ```
    # SSH to the ZED Box 
    ./docker_zedbox_bringup.sh
   ```
For launching visualization in remote PC:

    export ROS_DOMAIN_ID=2 # Adjust to your settings, default=0
    source install/setup.bash
    colcon build --packages-select soloco_launch # If not build
    ros2 launch soloco_launch remote_view.launch.py

## Simulation Example
Launch Locobot in Gazebo with GUI

    ros2 launch interbotix_xslocobot_sim xslocobot_gz_classic.launch.py robot_model:=locobot_base robot_name:=locobot use_lidar:=true use_rviz:=true use_gazebo_gui:=true 

Launch simultaneous localization and mapping using Nav2 and slam_toolbox

    ros2 launch interbotix_xslocobot_nav xslocobot_slam_toolbox.launch.py launch_driver:=false slam_mode:=online_async use_sim_time:=true cmd_vel_topic:=/locobot/diffdrive_controller/cmd_vel_unstamped
