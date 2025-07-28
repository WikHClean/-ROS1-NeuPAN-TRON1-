# PointCloud Transmission ROS Package

这个ROS包将点云传输系统集成到ROS Noetic中，提供实时点云数据和位置信息的发布功能。

## 功能特性

- 通过UDP接收点云数据并发布为ROS PointCloud2消息
- 接收位置和姿态数据并发布为ROS PoseStamped消息
- 自动发布TF变换（可选）
- 支持参数配置和话题重映射
- 提供RViz可视化配置

## 依赖项

确保已安装以下ROS包：
```bash
sudo apt-get install ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf2-ros
```

## 编译

1. 将此包放置在catkin工作空间的src目录下
2. 编译包：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 基本启动

使用默认参数启动节点：
```bash
roslaunch pointcloud_transmission pointcloud_transmission.launch
```

### 自定义参数启动

指定服务器IP和其他参数：
```bash
roslaunch pointcloud_transmission pointcloud_transmission.launch server_ip:=192.168.1.200 publish_rate:=20.0
```

### 启动并显示RViz

```bash
roslaunch pointcloud_transmission pointcloud_transmission.launch rviz:=true
```

## 参数说明

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `server_ip` | string | "192.168.1.100" | 数据服务器IP地址 |
| `frame_id` | string | "map" | 位置数据的父坐标系 |
| `child_frame_id` | string | "lidar" | 点云数据的子坐标系 |
| `publish_rate` | double | 10.0 | 发布频率(Hz) |
| `publish_tf` | bool | true | 是否发布TF变换 |

## 发布的话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/pointcloud` | sensor_msgs/PointCloud2 | 点云数据 |
| `/pose` | geometry_msgs/PoseStamped | 位置和姿态数据 |
| `/tf` | tf2_msgs/TFMessage | TF变换（如果启用） |

## 配置文件

- `config/pointcloud_transmission.yaml` - 节点参数配置
- `config/pointcloud_transmission.rviz` - RViz可视化配置

## 故障排除

1. **连接失败**：检查服务器IP地址和网络连接
2. **没有数据**：确认服务器正在发送数据
3. **编译错误**：检查依赖项是否正确安装

## 示例使用

查看点云数据：
```bash
rostopic echo /pointcloud
```

查看位置数据：
```bash
rostopic echo /pose
```

查看TF树：
```bash
rosrun tf2_tools view_frames.py
```

## 许可证

MIT License
