# new_vr_control_sim
# 0. 系统安装
将本文件夹放入ros2的工作空间的src中，如wk_space/src/coinrobot.

在本文件夹打开终端，执行以下指令
```bash
conda create -y -n coinrobot python=3.10 && conda activate coinrobot
pip install .
```
在ros2的工作空间，如wk_space/中执行以下指令
```
colcon build
```
# 1. 安装vr以及piper相关依赖
Install dependencies for ocluous quest 2

 ```shell
sudo apt install android-tools-adb
pip install pure-python-adb
pip install pyyaml
pip install rich
 ```

Type into `adb devices` and expect output like
```shell
List of devices attached
1WMHHA618C1506	device
```

Install dependencies for Piper

```shell
sudo apt update && sudo apt install ethtool
sudo apt update && sudo apt install can-utils
pip install catkin_pkg==0.4.23
pip install empy==3.3.4
pip install lark-parser==0.12.0
pip3 install python-can
pip3 install piper_sdk
```

If encounter the error of piper continuously using the python from system instead of your created conda env:
```shell
sudo apt purge 'python3-colcon-*' # remove the colcon build and reinstall it inside the conda env
conda activate tr3
pip install --upgrade colcon-common-extensions
which colcon # expected output: /home/zcai/miniforge3/envs/tr3/bin/colcon
```
# 2.安装pinocchio库
## 1. 安装依赖项
    sudo apt update
    sudo apt install -y \
        build-essential \
        cmake \
        pkg-config \
        libeigen3-dev \
        libboost-all-dev \
        liburdfdom-dev \
        liburdfdom-headers-dev \
        robot-state-publisher \
        ros-humble-rqt \
        ros-humble-rviz2 \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        python3-pip
    sudo apt install -y libhpp-fcl-dev
## 2. 安装ros下的pinocchio库
    sudo apt install -y libpinocchio-dev

# 3.操作步骤
## 1. 启动控制节点
    ros2 run my_pinocchio_robot my_node   --ros-args -p urdf_path:=/home/zjy/my_pinocchio_robot/src/piper_ros/src/piper_description/urdf/piper_description.urdf -p ee_frame:=link6 -p ik_tolerance:=0.001 -p ik_max_iter:=100
    其中urdf_path是需要控制的机械臂urdf路径，ee_frame是机械臂urdf中真实末端link名称，ik_tolerance控制逆解精度，ik_max_iter控制逆解最大迭代步数
## 2. 启动isaac sim并打开需要采集数据的场景usd文件
## 3. 启动vr node
    ros2 run vr_quest2_pub vr_pub

# 4. 注意事项
## 1. 使用python和ros的时候可能需要手动添加python的site package路径，因为ros路径可能覆盖conda路径
    export PYTHONPATH="/home/zjy/anaconda3/envs/tr3/lib/python3.10/site-packages:$PYTHONPATH" (修改为真实conda环境路径)
