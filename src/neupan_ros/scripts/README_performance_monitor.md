# NeuPAN 性能监控器使用说明

## 概述

这是一个独立的NeuPAN系统性能监控工具，可以实时监控系统的帧率、执行时间、CPU和内存使用率，无需修改原有的NeuPAN代码。

## 功能特性

### 📊 监控指标
- **帧率监控**: 监控控制命令(/cmd_vel)、激光扫描(/scan)、路径规划(/path)的发布频率
- **系统资源**: CPU使用率、内存占用、CPU频率
- **进程监控**: 自动检测和监控NeuPAN相关进程
- **实时统计**: 每5秒输出一次性能报告到控制台

### 📁 数据输出
- **JSON数据**: 详细的性能数据保存为JSON格式
- **可视化图表**: 自动生成性能趋势图表(PNG格式)
- **实时日志**: 控制台实时显示性能统计信息

## 安装和使用

### 1. 安装依赖

```bash
# 安装Python依赖
pip3 install psutil matplotlib numpy
```

### 2. 启动监控器

#### 方法一：使用启动脚本（推荐）
```bash
cd ~/neupan_ws/src/neupan_ros/scripts
chmod +x start_performance_monitor.sh
./start_performance_monitor.sh
```

#### 方法二：直接运行
```bash
# 设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/neupan_ws/devel/setup.bash

# 启动监控器
rosrun neupan_ros performance_monitor.py
```

### 3. 运行NeuPAN系统

在另一个终端中启动你的NeuPAN系统：
```bash
# 启动NeuPAN
roslaunch neupan_ros neupan_planner_real.launch
```

## 监控输出

### 控制台输出示例
```
============================================================
🔍 NeuPAN 性能监控报告
============================================================
📊 帧率统计:
   • 控制命令 (/cmd_vel): 50.12 Hz
   • 激光扫描 (/scan): 10.05 Hz
   • 路径规划 (/path): 5.23 Hz
💻 系统资源:
   • CPU使用率: 15.2% (平均: 12.8%)
   • 内存使用: 245.6 MB (平均: 230.1 MB)
   • CPU频率: 2400 MHz
🔧 NeuPAN进程数: 3
============================================================
```

### 文件输出

监控数据会自动保存到 `~/neupan_performance_logs/` 目录：

- **JSON数据文件**: `neupan_performance_YYYYMMDD_HHMMSS.json`
- **性能图表**: `neupan_performance_plot_YYYYMMDD_HHMMSS.png`

## 配置选项

你可以在 `performance_monitor.py` 中修改以下参数：

```python
# 配置参数
self.window_size = 100      # 保留最近100个数据点
self.log_interval = 5.0     # 每5秒输出一次统计
self.save_interval = 60.0   # 每60秒保存一次数据
```

## 监控的ROS话题

监控器会自动订阅以下话题：
- `/cmd_vel` - 控制命令
- `/scan` - 激光扫描数据
- `/neupan_path` - NeuPAN规划路径
- `/initial_path` - 初始路径

## 性能图表说明

生成的图表包含4个子图：
1. **帧率趋势**: 显示各个话题的帧率变化
2. **CPU使用率**: 显示CPU使用率随时间的变化
3. **内存使用**: 显示内存占用随时间的变化
4. **平均帧率对比**: 各话题平均帧率的柱状图对比

## 故障排除

### 1. 没有检测到NeuPAN进程
- 确保NeuPAN系统正在运行
- 检查进程名称是否包含"neupan"

### 2. 没有收到话题数据
- 检查ROS话题是否正确发布：`rostopic list`
- 确认话题名称是否匹配

### 3. 图表生成失败
- 确保安装了matplotlib：`pip3 install matplotlib`
- 检查输出目录权限

### 4. 依赖安装问题
```bash
# Ubuntu/Debian系统
sudo apt-get install python3-pip
pip3 install psutil matplotlib numpy

# 如果权限问题，使用用户安装
pip3 install --user psutil matplotlib numpy
```

## 停止监控

按 `Ctrl+C` 停止监控器，程序会自动保存最终的性能数据。

## 数据分析

### JSON数据结构
```json
{
  "timestamp": 1642123456.789,
  "datetime": "2022-01-14T10:30:56.789",
  "fps": {
    "cmd_vel": 50.12,
    "scan": 10.05,
    "path": 5.23
  },
  "system": {
    "cpu_percent": {
      "current": 15.2,
      "average": 12.8,
      "max": 20.1,
      "min": 8.5
    },
    "memory_mb": {
      "current": 245.6,
      "average": 230.1,
      "max": 280.3,
      "min": 210.8
    }
  },
  "process_count": 3
}
```

## 性能优化建议

根据监控结果，你可以：

1. **帧率过低**: 检查算法复杂度，优化计算密集型操作
2. **CPU使用率过高**: 考虑降低控制频率或优化算法
3. **内存使用过高**: 检查内存泄漏，优化数据结构
4. **进程数异常**: 检查是否有僵尸进程或重复启动

## 扩展功能

你可以根据需要扩展监控器：
- 添加更多ROS话题监控
- 增加网络I/O监控
- 添加磁盘使用监控
- 集成到ROS参数服务器
- 添加邮件或消息通知功能
