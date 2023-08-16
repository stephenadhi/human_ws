## Manual Installation
This documentation provides the instruction for manual installation. For ease of use, we provided a Dockerfile for both development and deployment.
1. Make a ROS 2 workspace folder and clone this repository inside src/ folder
    ```
    git clone --recursive https://github.com/stephenadhi/human_ws.git
    ```
2. Clone Interbotix Locobot packages
    ```
    git clone -b humble --recursive https://github.com/stephenadhi/interbotix_ros_core.git
    git clone -b humble https://github.com/stephenadhi/interbotix_ros_toolboxes.git
    git clone -b humble_odom_ground_truth https://github.com/stephenadhi/interbotix_ros_rovers.git
    git submodule update --init --recursive
    ```
2. Clone navigation2 from source (optional, necessary for using MPPI controller)
    ```
    git clone -b humble https://github.com/stephenadhi/navigation2.git
    ```

3. Clone Pedestrian Simulator (optional, otherwise create COLCON_IGNORE for pedsim_relay package)
    ```
    git clone -b humble https://github.com/stephenadhi/pedsim_ros.git
    ```
4. Install dependencies
    ```
    cd <your_workspace>
    rosdep install -i -y -r --from-paths src --rosdistro humble
    pip3 install -r requirements.txt
    ```

5. Build everything inside the workspace
   ```
    colcon build --packages-select zed_interfaces soloco_interfaces pedsim_msgs interbotix_xs_msgs interbotix_xs_driver dynamixel_workbench_toolbox
    source install/setup.bash
    colcon build
   ```

6. Define robot base type and model
   ```
   echo "export INTERBOTIX_XSLOCOBOT_BASE_TYPE=kobuki" >> ~/.bashrc
   echo "export INTERBOTIX_XSLOCOBOT_ROBOT_MODEL=locobot_base" >> ~/.bashrc
