#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, Quaternion, Point, Pose, Vector3
from std_msgs.msg import Header
import math
import time

class SimpleWebSocketOdometry:
    def __init__(self):
        rospy.init_node('simple_websocket_odometry', anonymous=True)
        
        # 获取参数
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.publish_rate = rospy.get_param('~publish_rate', 20.0)
        
        # 初始化位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 初始化速度
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 时间戳
        self.last_time = rospy.Time.now()
        
        # 发布器
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # TF广播器
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 订阅速度命令
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_odometry)
        
        rospy.loginfo("Simple WebSocket Odometry Publisher initialized")
        rospy.loginfo(f"Publishing odometry at {self.publish_rate} Hz")
        rospy.loginfo(f"Frames: {self.odom_frame} -> {self.base_frame}")
        
    def cmd_vel_callback(self, msg):
        """接收速度命令并更新内部速度状态"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
    def update_pose(self):
        """基于速度积分更新位姿"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt > 0:
            # 简单的运动学积分
            delta_x = self.linear_vel * math.cos(self.theta) * dt
            delta_y = self.linear_vel * math.sin(self.theta) * dt
            delta_theta = self.angular_vel * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # 保持角度在[-pi, pi]范围内
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.last_time = current_time
        
    def publish_odometry(self, event):
        """发布里程计消息和TF变换"""
        self.update_pose()
        
        current_time = rospy.Time.now()
        
        # 创建四元数
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        
        # 发布TF变换
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = odom_quat[0]
            t.transform.rotation.y = odom_quat[1]
            t.transform.rotation.z = odom_quat[2]
            t.transform.rotation.w = odom_quat[3]
            
            self.tf_broadcaster.sendTransform(t)
        
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 姿态
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        
        # 速度
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        
        # 协方差矩阵（简单设置）
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0,
                               0, 0, 0, 0, 0.1, 0,
                               0, 0, 0, 0, 0, 0.1]
        
        odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1]
        
        # 发布里程计消息
        self.odom_pub.publish(odom)
        
    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Simple WebSocket Odometry Publisher shutting down")

if __name__ == '__main__':
    try:
        node = SimpleWebSocketOdometry()
        node.run()
    except rospy.ROSInterruptException:
        pass
