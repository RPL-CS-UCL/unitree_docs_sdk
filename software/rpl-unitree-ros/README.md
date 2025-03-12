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

For the livox mid 360, you need to get the ID from a sticker on it. The IP address by default will end with the last 2 numbers in the ID, and this needs to be input into the .json config file in this repo.

Also, need to set static ip for your computer as it pushes via udp. Set it to 192.168.1.50.

For the realsense, jetson nano's have particular setups compared to other computers (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md), because of the jetpack os. Some people say that libuvc install works: https://github.com/IntelRealSense/librealsense/issues/13664. Other people downgrade to jetpack 6.0 and then patch the kernel.

```
catkin build rpl-unitree-ros -DUNISDK=ON
```

For this we will use Jetson Orin. At a high level we install the Nvdidia SDK Manager which allows us to flash the Jetson Orin. Then we connect keyboard/monitor and try our tests.


The SDK Manager install steps are [here](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html). Once this has been installed you will run it. You can plugin the jetson orin via usb c cable to a host computer running ubuntu with the SDK Manager installed. It also needs power via adaptor in box (plug it into back where dc power jack is, the usb c there seems to do the trick). For the Orin, steps to use SDK manager are [here](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/IN/QuickStart.html). 

I found that the Orin I had was already flashed with some stuff. Plugging in an HDMI monitor and keyboard and mouse for the first boot let me create a username/password and boot to Ubuntu. From here I connected to the Lab's wifi network (not eduroam) to get an IP address. The SDK Manager to do automatic flashing needs the IP address as well as the username/password (it does an SSH connection). I also didn't let the SDK manager install anything onto the host, as I had limited disk space. I only selected target stuff for the orin during that step.


If using a nano, setup for the nano is a bit different and you need to flash and SD card, steps [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write). 

On another computer connected to the same network:
```
xhost +local:root && docker run --rm -it --network host --nvidia --x11 --devices /dev/dri -e DISPLAY -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/tmp/.docker.xauth -v /tmp/.X11-units:/tmp/.X11-unix:rw -v ~/.Xauthority:/root/.Xauthority:rw -v /home/student/unitree_docs_sdk:/uni osrf/ros:noetic-desktop-full && xhost -local:root
export ROS_MASTER_URI=http://192.168.1.44:11311
```


For routing, you first have to set the the ethernet port to have priority to talk with livox. Then you can set the wifi to have prioritty to talk to the extra networrk computer. Thhen you can perhaps set softwwarre ethernet-
```
student@ubuntu:~$ sudo apt-get install ifmetric
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libglfw3 librealsense2 librealsense2-gl
Use 'sudo apt autoremove' to remove them.
The following NEW packages will be installed
  ifmetric
0 to upgrade, 1 to newly install, 0 to remove and 313 not to upgrade.
Need to get 11.0 kB of archives.
After this operation, 37.9 kB of additional disk space will be used.
Get:1 http://ports.ubuntu.com/ubuntu-ports jammy/universe arm64 ifmetric arm64 0.3-5 [11.0 kB]
Fetched 11.0 kB in 0s (205 kB/s)     
ydebconf: Delaying package configuration, since apt-utils is not installed.
Selecting previously unselected package ifmetric.
(Reading database ... 205632 files and directories currently installed.)
Preparing to unpack .../ifmetric_0.3-5_arm64.deb ...
Unpacking ifmetric (0.3-5) ...
Setting up ifmetric (0.3-5) ...
Processing triggers for man-db (2.10.2-1) ...
student@ubuntu:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.1.1     0.0.0.0         UG    600    0        0 wlP1p1s0
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 eno1
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.1.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
192.168.1.0     0.0.0.0         255.255.255.0   U     600    0        0 wlP1p1s0
student@ubuntu:~$ sudo ifmetric wlP1p1s0 50
student@ubuntu:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.1.1     0.0.0.0         UG    50     0        0 wlP1p1s0
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 eno1
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.1.0     0.0.0.0         255.255.255.0   U     50     0        0 wlP1p1s0
192.168.1.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
```

### jax on real robot
using python3.10
```python -m pip install jax[cuda12_local]==0.4.35.dev20241015+b076890 jaxlib==0.4.35.dev20241015 --index=https://pypi.jetson-ai-lab.dev/jp6/cu126
```

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