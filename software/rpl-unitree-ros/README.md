# rpl-go-1

This is a ros package to use the Go1 robot with RoboHike. It doesn't yet work, this is still in development.

![demo_screenshot](assets/system_demo.png)

## Quick Start
```
catkin build rpl-unitree-ros
source devel/setup.bash
roslaunch rpl-unitree-ros system_forest.launch
```

## CHAMP Controller

This is a controller from MIT, viewable here [here](https://github.com/chvmp/champ).

```
student@ubuntu20:~/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software$ git submodule update --init --recursive
sudo apt-get install ros-noetic-ecl-threads ros-noetic-robot-localization
```

```
catkin build -j2 go1_config champ champ_teleop champ_bringup champ_config
roslaunch go1_config bringup.launch rviz:=true
```

```
roslaunch go1_config bringup.launch rviz:=true description_file:=/home/student/robohike_ws/src/RPL-RoboHike/robot_docs_sdk/unitree_docs_sdk/software/unitree_ros/robots/go1_description/urdf/go1.urdf
```

```
roslaunch champ_teleop teleop.launch
```



## Attribution

The robot models all came from Unitree's ros repo on github [here](https://github.com/unitreerobotics/unitree_ros). It also uses the CMU NAV stack.

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