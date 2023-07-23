## Installation
1. Make a ROS 2 workspace folder and clone this repository inside src/ folder
    ```
    mkdir -p workspaces/humble/src
    git clone --recursive https://git.uni-due.de/locobot/human_ws.git
    git submodule update --init --recursive
    ```
2. Clone navigation2 from source (optional, necessary for using MPPI controller)
    ```
    git clone -b humble https://github.com/ros-planning/navigation2.git
    ```

5. Install ROS dependencies
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