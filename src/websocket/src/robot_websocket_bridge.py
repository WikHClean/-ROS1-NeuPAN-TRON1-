#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry  # 添加Odometry消息类型导入
import json
import uuid
import threading
import time
import websocket
import math

class RobotWebsocketBridge:
    def __init__(self):
        # ROS参数配置
        self.accid = rospy.get_param('~accid', "WF_TRON1A_234")
        self.ws_url = rospy.get_param('~ws_url', "ws://10.192.1.2:5000")
        
        # WebSocket客户端
        self.ws_client = None
        self.should_exit = False
        
        # 🔧 完全移除目标点相关变量 - WebSocket桥接器不参与导航管理
        # self.current_pose = None
        # self.goal_pose = None
        # self.reached_threshold = 0.2
        # self.slowdown_distance = 1.0
        # self.has_goal = False
        
        # ROS订阅者 - 只保留必要的命令订阅
        self.cmd_stand_sub = rospy.Subscriber('/robot/stand', Bool, self.stand_callback)
        self.cmd_walk_sub = rospy.Subscriber('/robot/walk', Bool, self.walk_callback)
        self.cmd_twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)  # 核心功能
        self.cmd_sit_sub = rospy.Subscriber('/robot/sit', Bool, self.sit_callback)
        self.cmd_stair_sub = rospy.Subscriber('/robot/stair_mode', Bool, self.stair_callback)
        self.cmd_stop_sub = rospy.Subscriber('/robot/emergency_stop', Bool, self.stop_callback)
        self.cmd_imu_sub = rospy.Subscriber('/robot/enable_imu', Bool, self.imu_callback)
        
        # 🚫 移除导航相关订阅 - 避免与NeuPAN冲突
        # self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # self.goal_sub = rospy.Subscriber('/move_base/goal', PoseStamped, self.goal_callback)
        
        # ROS发布者 - 只保留状态发布
        self.status_pub = rospy.Publisher('/robot/status', String, queue_size=50)
        # 保留cmd_vel_pub用于紧急停止等安全功能
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 启动WebSocket连接
        self.connect_websocket()
        
        rospy.loginfo("WebSocket桥接器启动 - 专注于速度命令转发，导航由NeuPAN管理")

    def generate_guid(self):
        return str(uuid.uuid4())

    def send_request(self, title, data=None):
        if data is None:
            data = {}
        
        message = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": self.generate_guid(),
            "data": data
        }

        message_str = json.dumps(message)
        
        if self.ws_client:
            try:
                self.ws_client.send(message_str)
            except Exception as e:
                rospy.logerr(f"WebSocket发送失败: {str(e)}")

    def connect_websocket(self):
        rospy.loginfo(f"正在连接到WebSocket服务器: {self.ws_url}")
        
        self.ws_client = websocket.WebSocketApp(
            self.ws_url,
            on_open=self.on_open,
            on_close=self.on_close,
            on_error=self.on_error
        )
        
        # 在单独线程中运行WebSocket客户端
        self.ws_thread = threading.Thread(target=self.ws_client.run_forever)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    # WebSocket回调函数
    def on_open(self, ws):
        rospy.loginfo("成功连接到机器人WebSocket接口")

    def on_close(self, ws, close_status_code, close_msg):
        rospy.logwarn(f"WebSocket连接关闭: {close_msg} (代码: {close_status_code})")

    def on_error(self, ws, error):
        rospy.logerr(f"WebSocket错误: {str(error)}")

    # ROS回调函数
    def twist_callback(self, msg):
        # 简化版本：直接转发速度命令，不依赖位姿信息
        
        # 基础速度倍数
        linear_factor = 1.0
        angular_factor = 1.5
        
        # 应用速度转换
        scaled_linear_x = msg.linear.x * linear_factor
        scaled_linear_y = msg.linear.y * linear_factor
        scaled_angular_z = msg.angular.z * angular_factor
        
        # 发送速度命令
        self.send_request("request_twist", {
            "x": scaled_linear_x,
            "y": scaled_linear_y,
            "z": scaled_angular_z
        })
        
        # 简化日志输出
        if abs(scaled_linear_x) > 0.01 or abs(scaled_linear_y) > 0.01 or abs(scaled_angular_z) > 0.01:
            rospy.loginfo(f"发送移动命令: 线速度[{scaled_linear_x:.2f}, {scaled_linear_y:.2f}] 角速度[{scaled_angular_z:.2f}]")

    def stand_callback(self, msg):
        if msg.data:
            self.send_request("request_stand_mode")
            rospy.loginfo("发送站立命令")

    def walk_callback(self, msg):
        if msg.data:
            self.send_request("request_walk_mode")
            rospy.loginfo("发送行走命令")

    def sit_callback(self, msg):
        if msg.data:
            self.send_request("request_sitdown")
            rospy.loginfo("发送坐下命令")

    def stair_callback(self, msg):
        self.send_request("request_stair_mode", {"enable": msg.data})
        rospy.loginfo(f"设置楼梯模式: {'启用' if msg.data else '禁用'}")

    def stop_callback(self, msg):
        if msg.data:
            self.send_request("request_emgy_stop")
            rospy.loginfo("发送紧急停止命令")

    def imu_callback(self, msg):
        self.send_request("request_enable_imu", {"enable": msg.data})
        rospy.loginfo(f"设置IMU: {'启用' if msg.data else '禁用'}")

    def shutdown(self):
        self.should_exit = True
        if self.ws_client:
            self.ws_client.close()
        if hasattr(self, 'ws_thread'):
            self.ws_thread.join()

if __name__ == '__main__':
    rospy.init_node('robot_websocket_bridge')
    bridge = RobotWebsocketBridge()
    rospy.on_shutdown(bridge.shutdown)
    rospy.spin()
