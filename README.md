# NeuPAN ROS

基于NeuPAN端到端算法的ROS导航系统，集成了重定位和路径规划功能。
全局路径规划使用A*算法，局部的避障以及路径规划功能使用neupan端到端算法
前期需使用原作者的[NeuPAN算法详细介绍](https://github.com/hanruihua/NeuPAN)仓库去基于自己的机器人模型的长宽去新训练一个模型
完成模型训练后需在yaml文件中正确加载机器人模型（模型文件后缀为.pth）

## 系统要求

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- 已安装 [NeuPAN Planner](https://github.com/hanruihua/neupan)

## 安装步骤

### 1. NeuPAN算法安装

```bash
git clone https://github.com/hanruihua/NeuPAN
cd NeuPAN
git checkout py38
pip install -e .
```
完成Neupan算法配置后，可以进行此项目的配置
```bash
git clone https://github.com/WikHClean/-ROS1-NeuPAN-TRON1-.git
cd neupan_ws\
```
之后进行第二步配置

### 2. 重定位环境配置

```bash
python3 -m pip install --upgrade pip
sudo apt install python3-dev python3-pip
pip install open3d scipy numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install numpy==1.24.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install thread -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 3. WebSocket环境配置

```bash
sudo apt install python3-dev python3-pip
sudo pip3 install websocket-client
```

### 4. ROS环境编译

```bash
cd ~/neupan_ws
catkin_make
```

## 使用方法

### 启动系统

```bash
source ~/neupan_ws/devel/setup.bash
roslaunch neupan_ros neupan_real_robot.launch
```

### 参数配置

- 如需修改参数，请编辑 `neupan_planner_real.yaml` 文件中的相关参数
- 详细的参数说明请参考配置文件中的注释

## 参考资料

- [neupan_ros详细介绍](https://github.com/hanruihua/neupan_ros)
- [NeuPAN算法详细介绍](https://github.com/hanruihua/NeuPAN)

感谢原作者的支持！

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

## 项目结构

```
neupan_ws/
├── src/
│   ├── neupan_ros/          # NeuPAN ROS包
│   └── relo/                # 重定位包
├── devel/                   # 编译输出
└── build/                   # 编译缓存
```
