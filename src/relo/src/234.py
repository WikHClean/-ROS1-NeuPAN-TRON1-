#!/usr/bin/env python3
# coding=utf8

import rospy
import rospkg
import ctypes
import numpy as np
import open3d as o3d
import threading
import time
import os
import signal
import subprocess
import yaml
import math
from scipy.spatial.transform import Rotation as R
from queue import Queue

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft
import tf2_ros

# ================= ROS 发布器和订阅器 =================
pointcloud_pub = None
pose_pub = None
map_pub = None
odom_pub = None
lidar_pub = None
scan_pub = None
tf_broadcaster = None

# ================= 全局变量 =================
initial_pose = np.eye(4)
global_map = None
trans = np.eye(4)
cur_scan = o3d.geometry.PointCloud()
in_cur_scan = o3d.geometry.PointCloud()
points = []
run_flag = True
icp_success = False
trans_1 = None  # 保存成功定位时的变换矩阵
first_success = False  # 记录是否首次成功定位


# 在全局作用域初始化频率统计变量
odom_stats = {
    'last_time': None,
    'count': 0,
    'total_count': 0,
    'last_log_time': time.time(),
    'start_time': time.time(),
    'freq': 0.0,
    'min_interval': float('inf'),
    'max_interval': 0.0
}
# ================= 常量定义 =================
MAP_VOXEL_SIZE = 0.1
MAX_POINTS = 300000
LOCALIZATION_TH = 0.995
LASER_SCAN_MIN_RANGE = 0.1
LASER_SCAN_MAX_RANGE = 30.0
LASER_SCAN_ANGLE_MIN = -math.pi
LASER_SCAN_ANGLE_MAX = math.pi
LASER_SCAN_ANGLE_INCREMENT = math.pi / 180.0  # 1度分辨率
Z_FILTER_MIN = -0.4
Z_FILTER_MAX = 0.4

# ================= 队列 =================
data_queue = Queue()
pos_data_queue = Queue()

# ================= 数据结构定义 =================
class PointCloudData(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float),
                ("r", ctypes.c_uint8), ("g", ctypes.c_uint8), ("b", ctypes.c_uint8), ("a", ctypes.c_uint8)]

class PosData(ctypes.Structure):
    _fields_ = [("pos_x", ctypes.c_float), ("pos_y", ctypes.c_float), ("pos_z", ctypes.c_float),
                ("qua_x", ctypes.c_float), ("qua_y", ctypes.c_float), ("qua_z", ctypes.c_float), ("qua_w", ctypes.c_float)]

# ================= 回调函数 =================
def initial_pose_callback(msg):
    """处理初始位姿估计"""
    global initial_pose
    quat = [msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w]
    trans = tft.quaternion_matrix(quat)
    trans[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    initial_pose = trans
    rospy.loginfo("更新初始位姿估计")

# ================= 点云处理函数 =================
def voxel_down_sample(pcd, voxel_size):
    """点云体素下采样"""
    try:
        return pcd.voxel_down_sample(voxel_size)
    except:
        return o3d.geometry.voxel_down_sample(pcd, voxel_size)

def initialize_global_map(map_cloud_filename):
    """加载并预处理全局地图"""
    global global_map
    try:
        rospy.loginfo(f"尝试加载地图: {map_cloud_filename}")
        global_map = o3d.io.read_point_cloud(map_cloud_filename)
        assert not global_map.is_empty(), "加载的地图为空"
        global_map = global_map.voxel_down_sample(MAP_VOXEL_SIZE)
        rospy.loginfo(f"成功加载地图，点云数量：{len(global_map.points)}")
    except Exception as e:
        rospy.logerr(f"地图加载失败: {str(e)}")
        raise RuntimeError(f"地图加载失败: {str(e)}")

def registration_at_scale(pc_scan, pc_map, initial, max_correspondence_distance=1.0):
    """点云配准函数"""
    # 使用兼容的Open3D API
    try:
        # 尝试新版本API
        result_icp = o3d.pipelines.registration.registration_generalized_icp(
            pc_scan, pc_map, max_correspondence_distance, initial,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
    except AttributeError:
        # 回退到旧版本API或ICP
        try:
            result_icp = o3d.pipelines.registration.registration_icp(
                pc_scan, pc_map, max_correspondence_distance, initial,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
            )
        except:
            # 最后的回退方案
            result_icp = o3d.registration.registration_icp(
                pc_scan, pc_map, max_correspondence_distance, initial,
                o3d.registration.TransformationEstimationPointToPoint(),
                o3d.registration.ICPConvergenceCriteria(max_iteration=50)
            )
    
    transformation = result_icp.transformation
    fitness = result_icp.fitness
    
    return fitness, transformation

def global_localization(pose_estimation, global_cur_scan):
    """全局定位函数"""
    global global_map
    fitness, transformation = registration_at_scale(
        pc_scan=global_cur_scan,
        pc_map=global_map,
        initial=pose_estimation,
        max_correspondence_distance=1.0
    )
    return (fitness > LOCALIZATION_TH), transformation

# ================= 数据转换函数 =================
def pointcloud_to_ros_msg(o3d_pc, frame_id="odom"):
    """Open3D点云转ROS消息"""
    points_np = np.asarray(o3d_pc.points)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    
    # 创建包含所有字段的点云消息
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.UINT8, 1),
        PointField('g', 13, PointField.UINT8, 1),
        PointField('b', 14, PointField.UINT8, 1),
        PointField('a', 15, PointField.UINT8, 1)
    ]
    
    # 创建点云数据
    points_with_color = []
    for i, point in enumerate(points_np):
        # 添加默认颜色（白色）
        r, g, b, a = 255, 255, 255, 255
        points_with_color.append((point[0], point[1], point[2], r, g, b, a))
    
    return pc2.create_cloud(header, fields, points_with_color)

def matrix_to_pose_msg(T):
    """转换矩阵转ROS位姿消息"""
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.position.x = T[0, 3]
    msg.pose.position.y = T[1, 3]
    msg.pose.position.z = T[2, 3]
    quat = tft.quaternion_from_matrix(T)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    return msg

def pos_data_to_matrix(pos_data):
    """位置数据转变换矩阵"""
    quat = R.from_quat([pos_data.qua_x, pos_data.qua_y, pos_data.qua_z, pos_data.qua_w])
    T = np.eye(4)
    T[:3, :3] = quat.as_matrix()
    T[:3, 3] = [pos_data.pos_x, pos_data.pos_y, pos_data.pos_z]
    return T

def create_laserscan_msg(points, frame_id="odom"):
    """从点云创建激光扫描消息"""
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = frame_id
    scan_msg.angle_min = LASER_SCAN_ANGLE_MIN
    scan_msg.angle_max = LASER_SCAN_ANGLE_MAX
    scan_msg.angle_increment = LASER_SCAN_ANGLE_INCREMENT
    scan_msg.time_increment = 0.0
    scan_msg.range_min = LASER_SCAN_MIN_RANGE
    scan_msg.range_max = LASER_SCAN_MAX_RANGE
    
    # 初始化360度扫描数组
    num_bins = int((LASER_SCAN_ANGLE_MAX - LASER_SCAN_ANGLE_MIN) / LASER_SCAN_ANGLE_INCREMENT)
    ranges = [float('inf')] * num_bins
    
    # 处理点云数据
    for point in points:
        x, y, z = point
        # 高度过滤
        if z < Z_FILTER_MIN or z > Z_FILTER_MAX:
            continue
            
        # 计算距离和角度
        range_val = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)
        
        # 过滤无效距离
        if range_val < LASER_SCAN_MIN_RANGE or range_val > LASER_SCAN_MAX_RANGE:
            continue
            
        # 计算索引
        bin_index = int((angle - LASER_SCAN_ANGLE_MIN) / LASER_SCAN_ANGLE_INCREMENT)
        
        # 确保索引在有效范围内
        if 0 <= bin_index < num_bins:
            # 保留最近距离
            if range_val < ranges[bin_index]:
                ranges[bin_index] = range_val
    
    # 二次过滤（清除0.3米内的点）
    ranges = [r if r > 0.2 else float('inf') for r in ranges]
    
    scan_msg.ranges = ranges
    return scan_msg

# ================= TF广播函数 =================
def broadcast_tf(T_device_to_current, timestamp):
    """广播TF变换（使用统一的时间戳）"""
    global trans, trans_1, first_success
    
    # 使用稍微提前的时间戳避免外推问题
    current_time = timestamp + rospy.Duration(0.1)  # 提前100ms
    
    # 确定要使用的变换矩阵：如果有保存的trans_1则优先使用
    broadcast_trans = trans_1 if first_success else trans
    
    # 1. map → odom：使用保存的变换矩阵
    map_to_odom = TransformStamped()
    map_to_odom.header.stamp = current_time
    map_to_odom.header.frame_id = "map"
    map_to_odom.child_frame_id = "odom"
    map_to_odom.transform.translation.x = broadcast_trans[0, 3]
    map_to_odom.transform.translation.y = broadcast_trans[1, 3]
    map_to_odom.transform.translation.z = broadcast_trans[2, 3]
    q = tft.quaternion_from_matrix(broadcast_trans)
    map_to_odom.transform.rotation.x = q[0]
    map_to_odom.transform.rotation.y = q[1]
    map_to_odom.transform.rotation.z = q[2]
    map_to_odom.transform.rotation.w = q[3]
    
    # 2. odom → base_footprint：设备当前位姿
    odom_to_base = TransformStamped()
    odom_to_base.header.stamp = current_time
    odom_to_base.header.frame_id = "odom"
    odom_to_base.child_frame_id = "base_link"
    odom_to_base.transform.translation.x = T_device_to_current[0, 3]
    odom_to_base.transform.translation.y = T_device_to_current[1, 3]
    odom_to_base.transform.translation.z = T_device_to_current[2, 3]
    q = tft.quaternion_from_matrix(T_device_to_current)
    odom_to_base.transform.rotation.x = q[0]
    odom_to_base.transform.rotation.y = q[1]
    odom_to_base.transform.rotation.z = q[2]
    odom_to_base.transform.rotation.w = q[3]
    
    # 发送两个变换
    tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

# ================= 数据获取函数 =================
def saving_points():
    """获取并保存点云数据（带频率统计）"""
    global cur_scan, points
    
    # 添加频率统计变量（使用函数属性代替全局变量）
    if not hasattr(saving_points, "last_time"):
        saving_points.last_time = time.time()
        saving_points.fetch_count = 0
        saving_points.start_time = time.time()
    
    cur_scan.clear()
    points.clear()
    
    # 记录获取点云的开始时间
    fetch_start = time.time()
    
    # 获取点云数据
    current_data, data_ptr = get_point_cloud_data()
    
    # 计算获取时延
    fetch_duration = time.time() - fetch_start
    
    # 处理点云
    for p in current_data[:MAX_POINTS]:
        points.append([p.x, p.y, p.z])
    cur_scan.points = o3d.utility.Vector3dVector(points)
    
    # 释放内存
    libmanifoldsdk.freePointCloudDataList(data_ptr)
    
    # 频率统计
    saving_points.fetch_count += 1
    current_time = time.time()
    elapsed = current_time - saving_points.last_time
    
    # 每1秒输出一次频率信息
    if elapsed >= 1.0:
        frequency = saving_points.fetch_count / elapsed
        total_elapsed = current_time - saving_points.start_time
        avg_frequency = saving_points.fetch_count / total_elapsed if total_elapsed > 0 else 0
        
        rospy.loginfo(
            f"点云获取频率: {frequency:.2f} Hz (瞬时) | "
            f"最近一帧获取耗时: {fetch_duration*1000:.2f} ms | "
            f"总帧数: {saving_points.fetch_count}"
        )
        
        # 重置计数器
        saving_points.fetch_count = 0
        saving_points.last_time = current_time
    
    return cur_scan, len(points)

def get_point_cloud_data():
    """从共享库获取点云数据"""
    data_ptr = ctypes.POINTER(PointCloudData)()
    size = ctypes.c_size_t()
    libmanifoldsdk.getAndClearPointCloudDataList(ctypes.byref(data_ptr), ctypes.byref(size))
    current_data = [data_ptr[i] for i in range(size.value)]
    return current_data, data_ptr

def get_pos_data():
    """从共享库获取位置数据"""
    data_ptr = ctypes.POINTER(PosData)()
    size = ctypes.c_size_t()
    libmanifoldsdk.GetPos(ctypes.byref(data_ptr), ctypes.byref(size))
    return [data_ptr[i] for i in range(size.value)]

# ================= 线程函数 =================
from copy import deepcopy

def point_cloud_data_thread():
    global cur_scan
    rospy.loginfo("点云数据线程已启动")
    count_total = 0
    
    # 新增的频率计算变量
    last_publish_time = time.time()
    publish_count = 0
    freq_interval = 1.0  # 频率计算间隔（秒）
    
    while run_flag:
        try:
            cur_scan, count = saving_points()
            if count > 1000:  # 从10000降低到1000，确保更频繁的激光扫描数据发布
                pointcloud_pub.publish(pointcloud_to_ros_msg(cur_scan))
                lidar_pub.publish(pointcloud_to_ros_msg(cur_scan, "odom"))
                
                data_queue.put((deepcopy(cur_scan), count))
                
                scan_msg = create_laserscan_msg(points, "odom")
                scan_pub.publish(scan_msg)
                
                count_total += 1
                
                # 新增的频率计算和打印
                publish_count += 1
                current_time = time.time()
                elapsed = current_time - last_publish_time
                
                if elapsed >= freq_interval:
                    freq = publish_count / elapsed
                    rospy.loginfo(f"LiDAR发布频率：{freq:.2f} Hz (最近{elapsed:.1f}秒内发布{publish_count}帧)")
                    publish_count = 0
                    last_publish_time = current_time
                
                if count_total % 10 == 0:
                    rospy.loginfo(f"已处理{count_total}帧点云数据，当前帧点数：{count}")
        except Exception as e:
            rospy.logerr(f"点云线程错误: {str(e)}")
        time.sleep(0.1)

def icp_counter_thread():
    global in_cur_scan, trans, icp_success, trans_1, first_success
    print("ICP定位线程已启动")
    icp_count = 0
    while run_flag:
        try:
            in_cur_scan.clear()  # 现在可以安全操作
            current_scan, count = data_queue.get()  # 解包队列数据
            in_cur_scan.points = current_scan.points  # 复制点云数据
            data_queue.queue.clear()
            if count > 10000:
                print(f"开始第{icp_count+1}次ICP定位计算，点云数量：{count}")
                initialized, trans = global_localization(initial_pose, in_cur_scan)
                icp_count += 1
                icp_success = initialized  # 更新ICP定位成功标志
                
                # ===== 新增部分：保存首次成功的变换矩阵 =====
                if initialized and not first_success:
                    trans_1 = trans.copy()  # 深拷贝当前变换矩阵
                    first_success = True
                    rospy.loginfo("首次重定位成功！保存变换矩阵为trans_1")
                # ========================================
                
                print(f"ICP定位结果：{'成功' if initialized else '失败'}，匹配度：{initialized}")
                print(f"当前位姿矩阵:\n{trans}")
                print(f"TF发布状态：{'已启用' if icp_success else '已禁用'}")
                print(f"当前变换矩阵类型：{'已保存trans_1' if first_success else '使用实时trans'}")
        except Exception as e:
            print(f"ICP线程出错：{str(e)}")

# ================= 主循环 =================
def main_loop():
    """主处理循环（带频率统计）"""
    global trans, icp_success, trans_1, first_success, odom_stats
    rospy.loginfo("主循环已启动，开始发布位姿")
    pose_count = 0
    last_pos = None
    
    while not rospy.is_shutdown() and run_flag:
        try:
            pos_data_array = get_pos_data()
            for pos_data in pos_data_array:
                # 转换原始位姿数据
                T_device_to_current = pos_data_to_matrix(pos_data)
                
                # ===== 修改部分：使用保存的变换矩阵计算 =====
                if first_success:
                    T_map_device = trans_1 @ T_device_to_current
                else:
                    T_map_device = trans @ T_device_to_current
                # =========================================
                
                # 发布设备位姿
                pose_msg = matrix_to_pose_msg(T_map_device)
                pose_pub.publish(pose_msg)
                
                # 发布Odometry消息
                odom_msg = Odometry()
                current_time = time.time()
                odom_msg.header.stamp = rospy.Time.from_sec(current_time)
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_footprint"
                odom_msg.pose.pose.position.x = pos_data.pos_x
                odom_msg.pose.pose.position.y = pos_data.pos_y
                odom_msg.pose.pose.position.z = pos_data.pos_z
                odom_msg.pose.pose.orientation.x = pos_data.qua_x
                odom_msg.pose.pose.orientation.y = pos_data.qua_y
                odom_msg.pose.pose.orientation.z = pos_data.qua_z
                odom_msg.pose.pose.orientation.w = pos_data.qua_w
                odom_pub.publish(odom_msg)
                
                # ===== 新增：里程计频率统计 =====
                # 更新统计计数
                odom_stats['count'] += 1
                odom_stats['total_count'] += 1
                
                # 第一次调用时初始化
                if odom_stats['last_time'] is None:
                    odom_stats['last_time'] = current_time
                    odom_stats['last_log_time'] = current_time
                    continue
                
                # 计算时间间隔
                interval = current_time - odom_stats['last_time']
                odom_stats['last_time'] = current_time
                
                # 更新最小/最大间隔
                if interval < odom_stats['min_interval']:
                    odom_stats['min_interval'] = interval
                if interval > odom_stats['max_interval']:
                    odom_stats['max_interval'] = interval
                
                # 计算瞬时频率
                if interval > 0.0001:
                    odom_stats['freq'] = 1.0 / interval
                
                # 每秒打印一次统计信息
                if current_time - odom_stats['last_log_time'] >= 1.0:
                    total_elapsed = current_time - odom_stats['start_time']
                    avg_freq = odom_stats['total_count'] / total_elapsed
                    
                    # 计算抖动（标准偏差估计）
                    avg_interval = total_elapsed / odom_stats['total_count']
                    jitter_estimate = (odom_stats['max_interval'] - odom_stats['min_interval']) * 1000
                    
                    rospy.loginfo(
                        f"[ODOM] 瞬时频率: {odom_stats['freq']:.1f}Hz | "
                        f"平均频率: {avg_freq:.1f}Hz | "
                        f"消息数: {odom_stats['count']} | "
                        f"最大间隔: {odom_stats['max_interval']*1000:.1f}ms | "
                        f"最小间隔: {odom_stats['min_interval']*1000:.1f}ms | "
                        f"抖动: {jitter_estimate:.1f}ms"
                    )
                    
                    # 重置计数器
                    odom_stats['count'] = 0
                    odom_stats['last_log_time'] = current_time
                    odom_stats['min_interval'] = float('inf')
                    odom_stats['max_interval'] = 0.0
                # ===== 频率统计结束 =====
                
                # 发布TF变换
                broadcast_tf(T_device_to_current, rospy.Time.from_sec(current_time))
                
                pose_count += 1
                if pose_count % 100 == 0:
                    current_pos = T_map_device[:3, 3].copy()
                    rospy.loginfo(f"已发布{pose_count}次位姿，当前位置：[{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}]")
                    rospy.loginfo(f"TF发布状态：{'已启用' if icp_success else '已禁用'}")
                    rospy.loginfo(f"使用{'首次保存' if first_success else '实时'}变换矩阵")
                    if last_pos is not None:
                        movement = np.linalg.norm(current_pos - last_pos)
                        rospy.loginfo(f"距离上次打印移动了：{movement:.3f}米")
                    last_pos = current_pos
        except Exception as e:
            rospy.logerr(f"主循环错误：{str(e)}")
            # 记录错误频率信息
            rospy.logerr(f"[ODOM] 错误前状态: "
                         f"瞬时频率: {odom_stats['freq']:.1f}Hz "
                         f"平均频率: {(odom_stats['total_count']/(time.time()-odom_stats['start_time'])):.1f}Hz")
        
        time.sleep(0.05)
# ================= 初始化函数 =================
def load_map_init():
    """加载并初始化地图"""
    global global_map
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('relocalization_pkg')
    
    # 从ROS参数服务器获取地图文件路径
    map_filename = rospy.get_param('/relocalization_node/map_config/map_filename', '')
    
    if not map_filename:
        rospy.logerr("未指定地图文件路径，请检查launch文件中的参数设置")
        raise ValueError("未指定地图文件路径")
    
    # 初始化全局地图
    initialize_global_map(map_filename)
    
    # 发布地图点云
    if map_pub and global_map:
        map_pub.publish(pointcloud_to_ros_msg(global_map, "map"))

def signal_handler(sig, frame):
    """信号处理函数"""
    global run_flag
    run_flag = False
    rospy.loginfo("接收到终止信号，关闭节点...")
    os._exit(0)

# ================= 主入口 =================
if __name__ == '__main__':
    rospy.loginfo("=== 重定位节点启动 ===")
    rospy.init_node('relocalization_node', log_level=rospy.INFO)
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化ROS发布器和订阅器
    rospy.loginfo("初始化ROS发布器和订阅器...")
    pointcloud_pub = rospy.Publisher('/device_pointcloud', PointCloud2, queue_size=10)
    pose_pub = rospy.Publisher('/device_pose', PoseStamped, queue_size=10)
    map_pub = rospy.Publisher('/map_pointcloud', PointCloud2, queue_size=10, latch=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    lidar_pub = rospy.Publisher('/livox/lidar', PointCloud2, queue_size=10)
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)  # 修复变量名
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rospy.Subscriber('/initial_pose', PoseStamped, initial_pose_callback)
    rospy.loginfo("ROS话题设置完成")

    # 加载地图
    load_map_init()

    # 加载共享库
    rospy.loginfo("加载共享库...")
    try:
        # 使用当前用户的路径
        lib_path = "/home/manifoldtech/neupan_ws/src/relo/src/liblxPC_test.so"
        libmanifoldsdk = ctypes.CDLL(lib_path)
        sendDataToServer = libmanifoldsdk.sendDataToServer
        receiveUdpData = libmanifoldsdk.receiveUdpData
        receivePosData = libmanifoldsdk.receivePosData
        rospy.loginfo(f"共享库加载成功: {lib_path}")
    except Exception as e:
        rospy.logerr(f"加载共享库失败: {str(e)}")
        raise

    # 设置共享库函数原型
    libmanifoldsdk.getAndClearPointCloudDataList.restype = ctypes.POINTER(PointCloudData)
    libmanifoldsdk.getAndClearPointCloudDataList.argtypes = [ctypes.POINTER(ctypes.POINTER(PointCloudData)), ctypes.POINTER(ctypes.c_size_t)]
    libmanifoldsdk.freePointCloudDataList.argtypes = [ctypes.POINTER(PointCloudData)]
    libmanifoldsdk.GetPos.restype = ctypes.POINTER(PosData)
    libmanifoldsdk.GetPos.argtypes = [ctypes.POINTER(ctypes.POINTER(PosData)), ctypes.POINTER(ctypes.c_size_t)]

    # 启动工作线程
    rospy.loginfo("启动工作线程...")
    try:
        threading.Thread(target=receiveUdpData, name="UDP接收线程").start()
        threading.Thread(target=receivePosData, name="位姿接收线程").start()
        threading.Thread(target=sendDataToServer, name="数据发送线程").start()
        threading.Thread(target=point_cloud_data_thread, name="点云处理线程").start()
        threading.Thread(target=icp_counter_thread, name="ICP定位线程").start()
        
        rospy.loginfo("所有线程启动完成，开始主循环...")
        main_loop()
    except Exception as e:
        rospy.logerr(f"线程启动失败: {str(e)}")
        raise
