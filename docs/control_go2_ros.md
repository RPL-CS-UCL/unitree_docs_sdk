### Start Using Unitree ROS2 Toolkits

First, replace `foxy` with the correct ROS2 distribution (e.g., `humble`).

1.  Navigate to your workspace and clone the Unitree ROS2 repository:

    ```shell
    cd robohike_ws/src/ros2_ws
    git clone https://github.com/unitreerobotics/unitree_ros2 -b
    cd unitree_ros2
    git checkout v1.0.0
    cd ../
    ```

2.  Install necessary tools and update packages:

    ```shell
    apt install gedit net-tools -y
    apt update && apt install ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl -y
    ```

3.  Clone and build CycloneDDS:

    ```shell
    cd unitree_ros2/cyclonedds_ws/src/
    git clone https://github.com/ros2/rmw_cyclonedds -b foxy
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
    cd ../
    colcon build --packages-select cyclonedds
    ```

4.  Source the ROS 2 setup and build the Unitree ROS2 packages:

    ```shell
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
    ```
