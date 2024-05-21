# Overview
**The main contribution of this project is to add an MPC controller to the official unitree_guide project, making it easier for students learning model-based control of quadruped robots to get started.**(本项目主要贡献为在宇树官方项目unitree_guide中加入MPC控制器，便于入门四足机器人基于模型控制的同学学习。)

**If my project is helpful for your learning, please light up a "Star" for me.**(如果我的项目对您的学习有所帮助，请为我点亮一颗“星星”。)
# quick start

## environment

1. Ubuntu 20.04
2. ROS Neotic

## build
Please refer to Yushu official configuration for dependencies.
1. [unitree_guide](https://github.com/unitreerobotics/unitree_guide)<br>
2. [unitree_ros](https://github.com/unitreerobotics/unitree_ros)<br>
3. [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real)

Clone this project
```
git clone git@github.com:Mr-Y-B-L/unitreeMPC_guide.git
```
Build
```
cd unitreeMPC_guide/
```
```
catkin_make
```
Running
```
source ./devel/setup.bash
```
Start Gazebo
```
roslaunch unitreeMPC_guide gazeboSim.launch
```
Start MPC controller
```
./devel/lib/unitreeMPC_guide/junior_ctrl
```
# Acknowledgments

[A1-QP-MPC-Controller](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller)