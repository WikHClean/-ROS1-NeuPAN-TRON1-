#!/bin/bash

# NeuPAN性能监控器启动脚本

echo "🚀 启动NeuPAN性能监控器..."

# 检查依赖
echo "📋 检查依赖..."

# 检查psutil
python3 -c "import psutil" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少psutil库，正在安装..."
    pip3 install psutil
fi

# 检查matplotlib
python3 -c "import matplotlib" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少matplotlib库，正在安装..."
    pip3 install matplotlib
fi

# 检查numpy
python3 -c "import numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少numpy库，正在安装..."
    pip3 install numpy
fi

echo "✅ 依赖检查完成"

# 设置环境
source /opt/ros/noetic/setup.bash
source ~/neupan_ws/devel/setup.bash

# 创建日志目录
mkdir -p ~/neupan_performance_logs

echo "📊 启动性能监控器..."
echo "📁 日志将保存到: ~/neupan_performance_logs"
echo "⏹️  按 Ctrl+C 停止监控"
echo ""

# 启动监控器
rosrun neupan_ros performance_monitor.py
