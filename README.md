# neupan_ros

## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- Python = 3.8
- Installed [NeuPAN Planner](https://github.com/hanruihua/neupan).

## Installation
1.1 NeuPAN安装
    - git clone https://github.com/hanruihua/NeuPAN
    - cd NeuPAN
    - git checkout py38
    - pip install -e . 

1.2 Relo_ws环境配置
    - python3 -m pip install --upgrade pip
    - sudo apt install python3-dev python3-pip
    - pip install open3d scipy numpy open3d -i https://pypi.tuna.tsinghua.edu.cn/simple
    - pip install numpy==1.24.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
    - pip install thread -i https://pypi.tuna.tsinghua.edu.cn/simple
    
1.3 Websocket环境配置
    - sudo apt install python3-dev python3-pip
    - sudo pip3 install websocket-client
    
1.4 ROS环境配置
    - cd ~/neupan_ws && catkin_make

2. 启动NeuPAN节点
    - source  ~/neupan_ws/devel/setup.bash
    - roslaunch neupan_ros neupan_real_robot.launch 

2.1 如需修改参数，可以修改neupan_planner_real.launch文件中的参数
2.2 neupan_ros的详细介绍请参考https://github.com/hanruihua/neupan_ros
    NeuPAN算法的详细介绍请参考https://github.com/hanruihua/NeuPAN，在此感谢作者的支持

