#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes
import std_msgs.msg

def create_cloud_with_fields(header, fields, points):
    """创建包含指定字段的点云消息"""
    cloud_struct = struct.Struct(" ".join(fields))
    
    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
    point_step, offset = cloud_struct.size, 0
    
    for i, point in enumerate(points):
        cloud_struct.pack_into(buff, offset, *point)
        offset += point_step
    
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=cloud_struct.size,
        row_step=cloud_struct.size * len(points),
        data=buff.raw
    )

class PointCloud2Converter:
    def __init__(self):
        rospy.init_node('pointcloud2_converter', anonymous=True)
        
        # 订阅标准PointCloud2消息
        self.sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.callback, queue_size=10)
        
        # 发布转换后的PointCloud2消息，包含Fast-LIO需要的字段
        self.pub = rospy.Publisher('/livox/lidar_processed', PointCloud2, queue_size=10)
        
        rospy.loginfo("PointCloud2 converter started - adding required fields for Fast-LIO")
        
    def callback(self, cloud_msg):
        # 提取点云数据
        points_list = []
        
        # 检查输入点云中的字段
        has_intensity = False
        for field in cloud_msg.fields:
            if field.name == 'intensity':
                has_intensity = True
                break
        
        # 提取点云数据并添加必要的字段
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity") if has_intensity else ("x", "y", "z"), skip_nans=True):
            # 如果原始点云有强度字段，使用它；否则设为默认值
            if has_intensity:
                x, y, z, intensity = p
            else:
                x, y, z = p
                intensity = 100  # 默认强度值
            
            # 添加Fast-LIO需要的字段
            time = 0.0  # 默认时间戳
            ring = 0    # 默认扫描线
            
            # 将点添加到列表
            points_list.append((x, y, z, intensity, time, ring))
        
        # 创建新的点云消息头
        header = std_msgs.msg.Header()
        header.stamp = cloud_msg.header.stamp
        header.frame_id = cloud_msg.header.frame_id
        
        # 定义点云字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1)
        ]
        
        # 创建新的点云消息
        cloud_processed = pc2.create_cloud(header, fields, points_list)
        
        # 发布转换后的消息
        self.pub.publish(cloud_processed)
        
if __name__ == '__main__':
    try:
        converter = PointCloud2Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
