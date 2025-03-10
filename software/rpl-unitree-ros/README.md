# rpl-go-1

This is a ros package to use the Go1 robot with RoboHike, and more specifically the CMU nav framework

![demo_screenshot](assets/nav_gif.gif)

## Quick Start

For the Go1:
```
mdkir -p ~/robohike_ws/src
git clone https://github.com/RPL-CS-UCL/RPL-RoboHike --recurse-submodules robohike_ws/src/RPL-RoboHike
cd robohike_ws
robo_hike_ws/src/RPL-RoboHike/src/baseline_navigation/cmu_autonomous_exploration_development/src/vehicle_simulator/mesh/download_environments.sh
catkin build rpl-unitree-ros
source devel/setup.bash
roslaunch rpl-unitree-ros system_forest.launch
```

For spot:
```
catkin build rpl-unitree-ros
source devel/setup.bash
roslaunch rpl-unitree-ros system_forest.launch robot_name:=spot_gazebo base_frame:=body controller_config_subpath:="/config/spot" urdf_package:=rpl-unitree-ros urdf_path_and_name:=xacro/spot.urdf.xacro
```

## Hardware Notes

For the livox mid 360, you need to get the ID from a sticker on it and put it in the .json config file in this repo.

Also, need to set static ip for your computer as it pushes via udp to 92.168.1.50.

## Attribution

The robot models and guidance on .xacro files came from Unitree's ros repo on github [here](https://github.com/unitreerobotics/unitree_ros). They also came from the spot_description repo [here](https://github.com/chvmp/spot_ros/tree/gazebo) and [here](https://github.com/heuristicus/spot_ros). It also uses the CMU NAV stack and CHAMP as the controller currently.

## Troubleshooting

### Spawn service failed

Error:

```
[ERROR] [1735467490.197852, 2972.804000]: Spawn service failed. Exiting.
[spawn_camera-7] process has died [pid 8269, exit code 1, cmd /opt/ros/noetic/lib/gazebo_ros/spawn_model -urdf -param /camera_description -model rgbd_camera __name:=spawn_camera __log:=/home/student/.ros/log/2fa2dda2-c5ce-11ef-829e-cd6a884391bb/spawn_camera-7.log].
log file: /home/student/.ros/log/2fa2dda2-c5ce-11ef-829e-cd6a884391bb/spawn_camera-7*.log
```

Why: if you are running on a computer with limited CPU it may have failed the timeout. You can check error message by looking at logs:

```
cat /home/student/.ros/log/2fa2dda2-c5ce-11ef-829e-cd6a884391bb/spawn_camera-7*.log
```
```
[INFO] [1735467490.196398, 2972.804000]: Spawn status: SpawnModel: Entity pushed to spawn queue, but spawn service timed out waiting for entity to appear in simulation under the name rgbd_camera
```

Solution:
ignore the error. This gazebo spawn service will process the spawn request in time, even after the spawn node times out and fails it will still be in the queue. You should still see the camera come up and be able to navigate.

### unitree_legged_msgs is not in the workspace
Error:
```
[build] Error: Given package 'unitree_legged_msgs' is not in the workspace and pattern does not match any package
```

```
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros/unitree_ros_to_real$ ls
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros/unitree_ros_to_real$ cd ..
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros$ git status
giHEAD detached at c20ca8f
nothing to commit, working tree clean
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros$ git submodule init
Submodule 'unitree_ros_to_real' (https://github.com/unitreerobotics/unitree_ros_to_real.git) registered for path 'unitree_ros_to_real'
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros$ git submodule update
Cloning into '/home/mz/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros/unitree_ros_to_real'...
Submodule path 'unitree_ros_to_real': checked out 'b989870124913091fbe75e0dcfb047eb4ca00a09'
```

### CMake Error, Could not find a package configuration file provided by "ecl_threads"
```
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "ecl_threads" with
```

Fix:
```
sudo apt-get install ros-noetic-ecl-threads*
```

### Multiple packages found with the same name "yocs_velocity_smoother"
```
[build] Error: There was an error while searching for available packages:

Multiple packages found with the same name "yocs_velocity_smoother":
- RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/yocs_velocity_smoother
- RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/yujin_ocs/yocs_velocity_smoother
```

Fix:
```
~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software$ rm -rf yocs_velocity_smoother/
```

### fatal error: champ/utils/urdf_loader.h: No such file or directory

```
Errors     << champ_base:make /home/mz/robohike_ws/logs/champ_base/build.make.002.log
In file included from /home/mz/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/champ/champ_base/src/message_relay.cpp:28:
/home/mz/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/champ/champ_base/include/message_relay.h:38:10: fatal error: champ/utils/urdf_loader.h: No such file or directory
   38 | #include <champ/utils/urdf_loader.h>
```

Fix:
```
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/champ/champ$ git submodule init
Submodule 'champ/include/champ' (https://github.com/chvmp/libchamp) registered for path 'include/champ'
mz@DESKTOP-3C29OTD:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/champ/champ$ git submodule update
Cloning into '/home/mz/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/champ/champ/include/champ'...
Submodule path 'include/champ': checked out '5572f50532d197c840db2fd283b84f3e1fdb4648'
```

## CMAKE error could not find tf2_sensor_msgs

```
Errors     << rpl-unitree-ros:check /home/mz/robohike_ws/logs/rpl-unitree-ros/build.check.028.log
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "tf2_sensor_msgs"
  with any of the following names:

    tf2_sensor_msgsConfig.cmake
    tf2_sensor_msgs-config.cmake

  Add the installation prefix of "tf2_sensor_msgs" to CMAKE_PREFIX_PATH or
  set "tf2_sensor_msgs_DIR" to a directory containing one of the above files.
  If "tf2_sensor_msgs" provides a separate development package or SDK, be
  sure it has been installed.
Call Stack (most recent call first):
  CMakeLists.txt:15 (find_package)
```

```
sudo apt-get install ros-noetic-tf2-sensor-msgs
```