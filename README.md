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

### 2.1 CMU navigation stack - [link](https://github.com/jizhang-cmu/autonomy_stack_go2)
```shell script
mkdir -p ~/ros2_ws/src
git clone https://github.com/jizhang-cmu/autonomy_stack_go2.git
```
And then follow this [README](https://github.com/jizhang-cmu/autonomy_stack_go2) to continue your installation.

### 2.2 Unitree Omniversal - [link](https://github.com/abizovnuralem/go2_omniverse)
