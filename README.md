# Unitree Document and SDK Collection

## 1. Structure
```shell script
anymal_docs_sdk
├── README.md
├── docker                         # docker files
├── docs                           # documents
```

## 2. Available Softwares
:exclamation: It should noted that Go2 only supports ROS2, while most of packages in RPL-RoboHike is are in ROS1. To avoid conflict between ROS1 and ROS2, please create another ROS2_ws and clone related packages in this workspace.

### 2.1 Unitree Document - [link](https://support.unitree.com/home/en/developer/about_Go2)
> NOTE by Jianhao: 
>  * If you only care about navigation, you can skip ```Application Development``` and ```Software Interface Services``` and turn **2.2** for software. 
>  * If you care about low-level control, you can goto ```Case reference``` for the interface

### 2.2 CMU navigation stack - [link](https://github.com/jizhang-cmu/autonomy_stack_go2)
```shell script
mkdir -p ~/ros2_ws/src
git clone https://github.com/jizhang-cmu/autonomy_stack_go2.git
```
You can try this if you want to deploy navigation on the robot (in the unity environment). 
Please follow this [README](https://github.com/jizhang-cmu/autonomy_stack_go2) to continue your installation.

### 2.3 Unitree GO2 Robot ROS2 - [link](https://github.com/dkanou/go2_robot)
You can try this if you want to setup the realsense and LiDAR on the real-world robot, and change the movement configuration.

### 2.4 Unitree Omniversal - [link](https://github.com/abizovnuralem/go2_omniverse)
You can try this if you want to use the Nvidia Issac Sim with the robot.

### 2.5 Unitree GO1, A1 Simulation ROS1 - [link](https://github.com/macc-n/ros_unitree)