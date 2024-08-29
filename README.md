# 全阵容兵种导航建图避障仿真方案（Beta版）
北京工业大学 PIP 战队 高颢嘉

## 致谢
感谢深圳北理莫斯科大学北极熊战队的开源，本项目基于其开源代码进行开发。\
https://gitee.com/SMBU-POLARBEAR/pb_rm_simulation

## 介绍
本项目基于 Ubuntu 22.04 系统，使用 ROS2 和 Gazebo 开发，实现全阵容兵种导航建图避障仿真。\
项目结构图：
![项目结构图](/docs/Structure.jpg)
仿真效果图：
![仿真效果图](/docs/gazebo.png)

## 项目优势
通过设置不同的 ROS_DOMAIN_ID 来隔离各个兵种的导航算法，保证各个兵种 ROS Topic 和 TF 树不会互相干扰。

## 快速开始
1. Ubuntu 22.04 系统
2. 安装 ROS2 Humble full desktop
3. 安装 Gazebo11
4. 安装 Livox-SDK2
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```
5. 克隆代码
```bash
git clone --recursive https://github.com/gaohaojia/PIP_RM_Simulation.git
cd PIP_RM_Simulation
```
6. 安装相关依赖
```bash
sudo rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
7. 编译
```bash
./toBuild.sh
```
8. 运行
```bash
./run.sh [robot_count]
```
将 [robot_count] 替换成要运行的机器人数量。

## 注意事项
1. 本项目对电脑 CPU 和内存要求较高，可通过降低仿真雷达的点云数量或降低运行机器人的数量来改善。
2. 对应机器人的导航算法需运行到对应的 ROS_DOMAIN_ID，否则会导致通信失败（运行前需设置环境变量 ROS_DOMAIN_ID）。
```bash
export ROS_DOMAIN_ID=[id]
```
3. 目前本项目只能发送 livox/lidar 、livox/imu 数据，并接受 cmd_vel 数据，其他数据可自行调整或联系我。

## 调整仿真参数
可在 simulation_bringup 包中的 launch 文件中调整仿真参数，如地图选择，每个兵种对应的 id，兵种初始位置等等。\
当前仿真环境预设了6台机器人，若想添加更多机器人，需对 simulation_bringup 包中的 launch 文件进行修改，并在 simulation_bringup 包中的 yaml 文件夹仿照其他机器人添加对应的 yaml 文件。