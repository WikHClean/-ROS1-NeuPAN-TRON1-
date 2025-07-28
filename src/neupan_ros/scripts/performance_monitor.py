#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
NeuPAN Performance Monitor
Independent performance monitoring script for NeuPAN system's frame rate, execution time, CPU and memory usage
No need to modify the original code, implemented through ROS topics and system monitoring
"""

import rospy
import psutil
import time
import threading
import json
import os
from collections import deque
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np

class NeuPANPerformanceMonitor:
    def __init__(self):
        rospy.init_node('neupan_performance_monitor', anonymous=True)
        
        # Configure parameters
        self.window_size = 100  # Keep the last 100 data points
        self.log_interval = 5.0  # Output statistics every 5 seconds
        self.save_interval = 30.0  # Save data every 60 seconds
        
        # Performance data storage
        self.frame_data = {
            'cmd_vel': deque(maxlen=self.window_size),
            'scan': deque(maxlen=self.window_size),
            'path': deque(maxlen=self.window_size)
        }
        
        self.timing_data = {
            'cmd_vel_intervals': deque(maxlen=self.window_size),
            'scan_intervals': deque(maxlen=self.window_size),
            'path_intervals': deque(maxlen=self.window_size)
        }
        
        # System resource monitoring
        self.neupan_processes = []
        self.system_stats = {
            'cpu_percent': deque(maxlen=self.window_size),
            'memory_mb': deque(maxlen=self.window_size),
            'cpu_freq': deque(maxlen=self.window_size)
        }
        
        # Timestamp recording
        self.last_timestamps = {
            'cmd_vel': None,
            'scan': None,
            'path': None
        }
        
        # Logging and saving
        self.stats_log = []
        self.last_log_time = time.time()
        self.last_save_time = time.time()
        
        # Create output directory
        self.output_dir = os.path.expanduser("~/neupan_performance_logs")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # ROS subscribers
        self.setup_subscribers()
        
        # Start monitoring thread
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_system_resources)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.loginfo("🚀 NeuPAN性能监控器已启动")
        rospy.loginfo(f"📊 数据将保存到: {self.output_dir}")
    
    def setup_subscribers(self):
        """设置ROS话题订阅者"""
        # 监控速度命令发布频率
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # 监控激光扫描频率
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        
        # 监控路径更新频率
        rospy.Subscriber("/neupan_path", Path, self.path_callback)
        rospy.Subscriber("/initial_path", Path, self.initial_path_callback)
        
        rospy.loginfo("✅ ROS话题订阅者已设置")
        
        # 添加调试信息 - 检查话题是否存在
        import subprocess
        try:
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
            available_topics = result.stdout.strip().split('\n')
            rospy.loginfo("🔍 当前可用的ROS话题:")
            for topic in available_topics:
                if any(keyword in topic for keyword in ['cmd_vel', 'scan', 'path']):
                    rospy.loginfo(f"   ✅ {topic}")
        except Exception as e:
            rospy.logwarn(f"无法获取话题列表: {e}")
    
    def cmd_vel_callback(self, msg):
        """Velocity command callback - Monitor control frequency"""
        current_time = time.time()
        self.frame_data['cmd_vel'].append(current_time)
        
        # 添加调试信息
        rospy.loginfo_throttle(10.0, "📡 收到 /cmd_vel 消息")
        
        if self.last_timestamps['cmd_vel'] is not None:
            interval = current_time - self.last_timestamps['cmd_vel']
            self.timing_data['cmd_vel_intervals'].append(interval)
        
        self.last_timestamps['cmd_vel'] = current_time
    
    def scan_callback(self, msg):
        """Laser scan callback - Monitor sensor frequency"""
        current_time = time.time()
        self.frame_data['scan'].append(current_time)
        
        # 添加调试信息
        rospy.loginfo_throttle(10.0, "📡 收到 /scan 消息")
        
        if self.last_timestamps['scan'] is not None:
            interval = current_time - self.last_timestamps['scan']
            self.timing_data['scan_intervals'].append(interval)
        
        self.last_timestamps['scan'] = current_time
    
    def path_callback(self, msg):
        """Path callback - Monitor path planning frequency"""
        current_time = time.time()
        self.frame_data['path'].append(current_time)
        
        if self.last_timestamps['path'] is not None:
            interval = current_time - self.last_timestamps['path']
            self.timing_data['path_intervals'].append(interval)
        
        self.last_timestamps['path'] = current_time
    
    def initial_path_callback(self, msg):
        """Initial path callback"""
        rospy.loginfo_throttle(5.0, f"📍 初始路径已更新，包含 {len(msg.poses)} 个点")
    
    def _find_neupan_processes(self):
        """Find NeuPAN-related processes"""
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                if 'neupan' in cmdline.lower() or 'neupan' in proc.info['name'].lower():
                    processes.append(proc)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return processes
    
    def _monitor_system_resources(self):
        """System resource monitoring thread"""
        # Get system information (only on the first run)
        if not hasattr(self, 'cpu_count'):
            self.cpu_count = psutil.cpu_count()
            self.cpu_count_logical = psutil.cpu_count(logical=True)
            rospy.loginfo(f"🖥️  系统信息: {self.cpu_count} 个核心 ({self.cpu_count_logical} 个逻辑核心)")
        
        while self.monitoring:
            try:
                # Update NeuPAN process list
                if not self.neupan_processes or len(self.neupan_processes) == 0:
                    self.neupan_processes = self._find_neupan_processes()
                
                # Calculate total CPU and memory usage of NeuPAN processes
                total_cpu_raw = 0.0
                total_memory = 0.0
                
                valid_processes = []
                for proc in self.neupan_processes:
                    try:
                        cpu_percent = proc.cpu_percent()
                        memory_mb = proc.memory_info().rss / 1024 / 1024
                        total_cpu_raw += cpu_percent
                        total_memory += memory_mb
                        valid_processes.append(proc)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                
                self.neupan_processes = valid_processes
                
                # Convert CPU usage to system-wide percentage
                # e.g., 4-core system, 400% raw value converted to 100% system usage
                total_cpu_percent = (total_cpu_raw / self.cpu_count_logical) if self.cpu_count_logical > 0 else 0.0
                
                # Record system statistics
                self.system_stats['cpu_percent'].append(total_cpu_percent)
                self.system_stats['memory_mb'].append(total_memory)
                
                # CPU frequency
                try:
                    cpu_freq = psutil.cpu_freq().current if psutil.cpu_freq() else 0
                    self.system_stats['cpu_freq'].append(cpu_freq)
                except:
                    self.system_stats['cpu_freq'].append(0)
                
                time.sleep(1.0)
                
            except Exception as e:
                rospy.logwarn(f"系统资源监控错误: {e}")
                time.sleep(1.0)
    
    def calculate_fps(self, topic_name):
        """Calculate frame rate for a given topic"""
        if topic_name not in self.timing_data or len(self.timing_data[f'{topic_name}_intervals']) < 2:
            return 0.0
        
        intervals = list(self.timing_data[f'{topic_name}_intervals'])
        if not intervals:
            return 0.0
        
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else 0.0
    
    def get_system_stats(self):
        """Get system statistics"""
        stats = {}
        
        for key, values in self.system_stats.items():
            if values:
                stats[key] = {
                    'current': values[-1],
                    'average': sum(values) / len(values),
                    'max': max(values),
                    'min': min(values)
                }
            else:
                stats[key] = {'current': 0, 'average': 0, 'max': 0, 'min': 0}
        
        return stats
    
    def log_performance_stats(self):
        """Output performance statistics"""
        current_time = time.time()
        
        if (current_time - self.last_log_time) < self.log_interval:
            return
        
        # Calculate frame rates
        cmd_vel_fps = self.calculate_fps('/cmd_vel')
        scan_fps = self.calculate_fps('/scan')
        path_fps = self.calculate_fps('/path')
        
        # Get system statistics
        sys_stats = self.get_system_stats()
        
        # Create statistics record
        stats_record = {
            'timestamp': current_time,
            'datetime': datetime.now().isoformat(),
            'fps': {
                'cmd_vel': cmd_vel_fps,
                'scan': scan_fps,
                'path': path_fps
            },
            'system': sys_stats,
            'process_count': len(self.neupan_processes)
        }
        
        self.stats_log.append(stats_record)
        
        # Output to console
        rospy.loginfo("=" * 60)
        rospy.loginfo("🔍 NeuPAN性能监控器报告")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📊 帧率统计:")
        rospy.loginfo(f"   • 控制命令 (/cmd_vel): {cmd_vel_fps:.2f} Hz")
        rospy.loginfo(f"   • 激光扫描 (/scan): {scan_fps:.2f} Hz")
        rospy.loginfo(f"   • 路径规划 (/path): {path_fps:.2f} Hz")
        rospy.loginfo(f"💻 系统资源:")
        rospy.loginfo(f"   • CPU 使用率: {sys_stats['cpu_percent']['current']:.1f}% "
                     f"(平均: {sys_stats['cpu_percent']['average']:.1f}%)")
        rospy.loginfo(f"   • 内存使用量: {sys_stats['memory_mb']['current']:.1f} MB "
                     f"(平均: {sys_stats['memory_mb']['average']:.1f} MB)")
        rospy.loginfo(f"   • CPU 频率: {sys_stats['cpu_freq']['current']:.0f} MHz")
        rospy.loginfo(f"🔧 NeuPAN 进程数量: {len(self.neupan_processes)}")
        rospy.loginfo("=" * 60)
        
        self.last_log_time = current_time
    
    def save_performance_data(self):
        """Save performance data to file"""
        current_time = time.time()
        
        if (current_time - self.last_save_time) < self.save_interval:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save JSON data
        json_filename = os.path.join(self.output_dir, f"neupan_performance_{timestamp}.json")
        try:
            with open(json_filename, 'w', encoding='utf-8') as f:
                json.dump(self.stats_log, f, indent=2, ensure_ascii=False)
            rospy.loginfo(f"📁 性能数据已保存: {json_filename}")
        except Exception as e:
            rospy.logerr(f"保存 JSON 数据失败: {e}")
        
        # Generate performance plots
        self.generate_performance_plots(timestamp)
        
        self.last_save_time = current_time
    
    def generate_performance_plots(self, timestamp):
        """Generate performance plots"""
        try:
            if len(self.stats_log) < 2:
                return
            
            # Extract data
            times = [datetime.fromisoformat(record['datetime']) for record in self.stats_log]
            cmd_vel_fps = [record['fps']['cmd_vel'] for record in self.stats_log]
            scan_fps = [record['fps']['scan'] for record in self.stats_log]
            path_fps = [record['fps']['path'] for record in self.stats_log]
            cpu_usage = [record['system']['cpu_percent']['current'] for record in self.stats_log]
            memory_usage = [record['system']['memory_mb']['current'] for record in self.stats_log]
            
            # Create plots
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
            fig.suptitle('NeuPAN Performance Monitor Report', fontsize=16, fontweight='bold')
            
            # Frame rate plot
            ax1.plot(times, cmd_vel_fps, 'b-', label='Control Commands', linewidth=2)
            ax1.plot(times, scan_fps, 'g-', label='Laser Scan', linewidth=2)
            ax1.plot(times, path_fps, 'r-', label='Path Planning', linewidth=2)
            ax1.set_title('Frame Rate (Hz)')
            ax1.set_ylabel('Frame Rate (Hz)')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # CPU usage plot
            ax2.plot(times, cpu_usage, 'orange', linewidth=2)
            ax2.set_title('CPU Usage (%)')
            ax2.set_ylabel('CPU %')
            ax2.grid(True, alpha=0.3)
            
            # Memory usage plot
            ax3.plot(times, memory_usage, 'purple', linewidth=2)
            ax3.set_title('Memory Usage (MB)')
            ax3.set_ylabel('Memory (MB)')
            ax3.grid(True, alpha=0.3)
            
            # Average frame rate comparison plot
            ax4.bar(['Control Cmd', 'Laser Scan', 'Path Plan'], 
                   [np.mean(cmd_vel_fps), np.mean(scan_fps), np.mean(path_fps)],
                   color=['blue', 'green', 'red'], alpha=0.7)
            ax4.set_title('Average Frame Rate Comparison')
            ax4.set_ylabel('Average FPS (Hz)')
            
            # Adjust layout and save
            plt.tight_layout()
            plot_filename = os.path.join(self.output_dir, f"neupan_performance_plot_{timestamp}.png")
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            plt.close()
            
            rospy.loginfo(f"📈 性能图已生成: {plot_filename}")
            
        except Exception as e:
            rospy.logerr(f"生成性能图失败: {e}")
    
    def run(self):
        """Main running loop"""
        rate = rospy.Rate(10)  # 10Hz monitoring frequency
        
        rospy.loginfo("🎯 性能监控循环已启动")
        
        try:
            while not rospy.is_shutdown():
                self.log_performance_stats()
                self.save_performance_data()
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 收到中断信号，停止监控...")
        finally:
            self.stop()
    
    def stop(self):
        """Stop monitoring"""
        self.monitoring = False
        
        # Save final data
        if self.stats_log:
            final_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_performance_data()
            rospy.loginfo("💾 最终性能数据已保存")
        
        # Wait for monitoring thread to finish
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        rospy.loginfo("✅ 性能监控器已停止")

def main():
    try:
        monitor = NeuPANPerformanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 节点中断")
    except Exception as e:
        rospy.logerr(f"性能监控器错误: {e}")

if __name__ == '__main__':
    main()
