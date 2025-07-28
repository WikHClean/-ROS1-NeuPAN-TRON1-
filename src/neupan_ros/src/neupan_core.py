#!/usr/bin/env python3

"""
neupan_core is the main class for the neupan_ros package. It is used to run the NeuPAN algorithm in the ROS framework, which subscribes to the laser scan and localization information, and publishes the velocity command to the robot.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

from neupan import neupan
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from math import sin, cos, atan2
import numpy as np
from neupan.util import get_transform
import tf
import sensor_msgs.point_cloud2 as pc2


class neupan_core:
    def __init__(self) -> None:

        rospy.init_node("neupan_node", anonymous=True)

        # ros parameters
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_link")
        self.marker_size = float(rospy.get_param("~marker_size", "0.05"))
        self.marker_z = float(rospy.get_param("~marker_z", "1.0"))

        scan_angle_range_para = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        # 确保参数是字符串类型，处理可能的数值类型参数
        scan_angle_range_para = str(scan_angle_range_para)
        # 处理单个数值的情况（如360度）
        if len(scan_angle_range_para.split()) == 1:
            angle_val = float(scan_angle_range_para)
            if angle_val == 360:
                # 360度表示全范围扫描
                self.scan_angle_range = np.array([-3.14159, 3.14159], dtype=np.float32)
            else:
                # 假设是对称范围
                angle_rad = angle_val * 3.14159 / 180.0  # 转换为弧度
                self.scan_angle_range = np.array([-angle_rad/2, angle_rad/2], dtype=np.float32)
        else:
            # 修复NumPy兼容性问题：使用array + split替代fromstring
            self.scan_angle_range = np.array(
                [float(x) for x in scan_angle_range_para.split()], dtype=np.float32
            )

        self.scan_downsample = int(rospy.get_param("~scan_downsample", "1"))

        scan_range_para = rospy.get_param("~scan_range", "0.0, 5.0")
        # 确保参数是字符串类型，处理可能的数值类型参数
        scan_range_para = str(scan_range_para)
        # 处理单个数值的情况
        if len(scan_range_para.replace(",", " ").split()) == 1:
            max_range = float(scan_range_para)
            # 单个值表示最大距离，最小距离设为0.1
            self.scan_range = np.array([0.1, max_range], dtype=np.float32)
        else:
            # 修复NumPy兼容性问题：使用array + split替代fromstring
            self.scan_range = np.array(
                [float(x) for x in scan_range_para.replace(",", " ").split()], dtype=np.float32
            )

        self.dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", True)
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param("~include_initial_path_direction", True)
        
        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        pan = {'dune_checkpoint': self.dune_checkpoint}
        
        # 检查是否是绝对路径，如果是，直接使用
        if self.planner_config_file.startswith('/'):
            config_path = self.planner_config_file
        else:
            config_path = neupan.util.file_check(self.planner_config_file)
            
        self.neupan_planner = neupan.init_from_yaml(
            config_path, pan=pan
        )
        # print()

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False
        self.arrive = False
        self.force_movement_after_goal = False
        # 🔧 初始化路径更新时间和宽限期
        self.path_updated_time = 0.0  # 初始化为0而不是None
        self.reset_grace_period = 1.0  # 1秒宽限期

        # publisher
        self.vel_pub = rospy.Publisher("/neupan_cmd_vel", Twist, queue_size=10)
        self.plan_pub = rospy.Publisher("/neupan_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/neupan_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/neupan_initial_path", Path, queue_size=10
        )  # initial path

        ## for rviz visualization
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )

        self.listener = tf.TransformListener()

        # subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # three types of initial path:
        # 1. from given path
        # 2. from waypoints
        # 3. from goal position
        # 注意：/move_base_simple/goal 已经在launch文件中重映射到 /neupan_goal
        # 所以这里只需要订阅 /neupan_goal 即可，避免重复处理
        rospy.Subscriber("/initial_path", Path, self.path_callback)
        rospy.Subscriber("/neupan_waypoints", Path, self.waypoints_callback)
        rospy.Subscriber("/neupan_goal", PoseStamped, self.goal_callback)
        
    def run(self):

        r = rospy.Rate(50)

        while not rospy.is_shutdown():

            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.map_frame, self.base_frame, rospy.Time(0)
                )

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf for the transform from {} to {}".format(
                        self.base_frame, self.map_frame
                    ),
                )
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(1, "waiting for robot state")
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            if (
                len(self.neupan_planner.waypoints) >= 1
                and self.neupan_planner.initial_path is None
            ):
                self.neupan_planner.set_initial_path_from_state(self.robot_state)
                # print('set initial path', self.neupan_planner.initial_path)

            if self.neupan_planner.initial_path is None:
                rospy.logwarn_throttle(1, "waiting for neupan initial path")
                continue

            rospy.loginfo_once("initial Path Received")
            self.ref_path_pub.publish(
                self.generate_path_msg(self.neupan_planner.initial_path)
            )

            if self.obstacle_points is None:
                rospy.logwarn_throttle(
                    1, "No obstacle points, only path tracking task will be performed"
                )

            action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

            # 检查是否在路径更新后的宽限期内
            current_time = rospy.get_time()
            in_grace_period = (self.path_updated_time is not None and 
                             current_time - self.path_updated_time < self.reset_grace_period)
            
            if in_grace_period:
                # 在宽限期内，不允许NeuPAN设置arrive状态，防止阻止速度发布
                self.stop = info.get("stop", False)
                # self.arrive保持False，忽略NeuPAN的arrive状态
                rospy.loginfo_throttle(1.0, f"Grace period active ({current_time - self.path_updated_time:.1f}s) - ignoring arrive state")
            else:
                # 正常情况下，接受NeuPAN的状态
                self.stop = info.get("stop", False)
                self.arrive = info.get("arrive", False)

            # 只有在非宽限期且真正到达时才显示到达信息
            if info.get("arrive", False) and not in_grace_period:
                rospy.loginfo_throttle(0.1, "arrive at the target")
                
            # publish the path and velocity
            if "opt_state_list" in info:
                self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
            else:
                rospy.logwarn_throttle(1.0, "opt_state_list not available")
                
            if "ref_state_list" in info:
                self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
            else:
                rospy.logwarn_throttle(1.0, "ref_state_list not available")
                
            self.vel_pub.publish(self.generate_twist_msg(action))

            self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
            self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

            if info.get("stop", False):
                rospy.logwarn_throttle(
                    0.5,
                    "neupan stop with the min distance "
                    + str(self.neupan_planner.min_distance.detach().item())
                    + " threshold "
                    + str(self.neupan_planner.collision_threshold),
                )

            r.sleep()

    # scan callback
    def scan_callback(self, scan_msg):

        if self.robot_state is None:
            return None

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []
        # x, y, z, yaw, pitch, roll = self.lidar_offset

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.scan_downsample == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            rospy.loginfo_once("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.map_frame, self.lidar_frame, rospy.Time(0)
            )

            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            rospy.loginfo_once("Scan obstacle points Received")

            return self.obstacle_points

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo_throttle(
                1,
                "waiting for tf for the transform from {} to {}".format(
                    self.lidar_frame, self.map_frame
                ),
            )
            return

    def path_callback(self, path):

        initial_point_list = []

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo_throttle(0.1, "🛤️ 从给定路径更新初始路径")
            
            # 🔧 重置状态但不重置规划器，避免清除刚设置的路径
            self.stop = False
            self.arrive = False
            rospy.loginfo("✅ 重置stop和arrive状态")
            
            # 🔍 调试：检查路径设置前的状态
            rospy.loginfo(f"🔍 设置路径前 - initial_path是否为None: {self.neupan_planner.initial_path is None}")
            rospy.loginfo(f"🔍 准备设置的路径点数量: {len(initial_point_list)}")
            
            # 设置新的初始路径
            self.neupan_planner.set_initial_path(initial_point_list)
            
            # 🔍 调试：检查路径设置后的状态
            rospy.loginfo(f"🔍 设置路径后 - initial_path是否为None: {self.neupan_planner.initial_path is None}")
            
            # 🚫 移除有问题的reset()调用 - 它会清除刚设置的路径
            # 只在必要时进行轻量级重置，不影响路径数据
            # self.neupan_planner.reset()  # 注释掉这行
            
            # 🔧 替代方案：只重置必要的内部状态，不清除路径
            # 如果NeuPAN有专门的状态重置方法，可以在这里调用
            # 例如：self.neupan_planner.reset_states_only()
            
            # 记录路径更新时间 - 启动宽限期
            self.path_updated_time = rospy.get_time()
            
            rospy.loginfo("🚀 路径设置完成 - 保留路径数据，只重置状态")

    def waypoints_callback(self, path):
        
        '''
        Utilize multiple waypoints (goals) to set the initial path
        '''

        waypoints_list = [self.robot_state]

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            rospy.loginfo_throttle(0.1, "initial path update from waypoints")
            self.neupan_planner.update_initial_path_from_waypoints(waypoints_list)
            self.neupan_planner.reset()
            
            # 记录路径更新时间
            self.path_updated_time = rospy.get_time()

    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        print(f"🎯 NEW GOAL RECEIVED: {[x, y, theta]}")
        
        # 🔧 强力状态重置 - 确保机器人能够响应新目标
        self.stop = False
        self.arrive = False
        rospy.loginfo("✅ Reset stop and arrive states for new goal")
        
        # 🔄 强力NeuPAN重置 - 多次重置确保清除所有内部状态
        rospy.loginfo("🔄 Performing COMPREHENSIVE NeuPAN reset...")
        
        # 清除所有可能的内部状态
        self.neupan_planner.reset()
        
        # 🚫 移除有问题的直接属性赋值 - 这些属性可能是只读的
        # 改为依赖NeuPAN内部的reset()方法来清理状态
            
        # 多次重置以确保彻底清除状态
        for i in range(3):
            self.neupan_planner.reset()
            rospy.loginfo(f"🔄 NeuPAN reset iteration {i+1}/3")

        rospy.loginfo("🛤️ Updating initial path from goal position")
        self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        
        # 再次重置以确保新路径被正确加载
        self.neupan_planner.reset()
        
        # 记录路径更新时间 - 启动宽限期
        self.path_updated_time = rospy.get_time()
        
        rospy.loginfo("🚀 COMPREHENSIVE GOAL RESET COMPLETE - READY FOR NEW NAVIGATION!")
        rospy.loginfo(f"🕐 Grace period started: {self.reset_grace_period} seconds")

    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    # generate ros message
    def generate_path_msg(self, path_list):

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0

        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.seq = index

            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])

            path.poses.append(ps)

        return path

    def generate_twist_msg(self, vel):

        if vel is None:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        # 🔧 检查是否在宽限期内 - 如果是，忽略stop/arrive状态
        current_time = rospy.get_time()
        in_grace_period = (current_time - self.path_updated_time) < self.reset_grace_period
        
        # 在宽限期内，允许运动；否则检查stop/arrive状态
        if not in_grace_period and (self.stop or self.arrive):
            rospy.loginfo_throttle(1.0, f"🛑 速度被阻止 - stop: {self.stop}, arrive: {self.arrive}")
            return Twist()
        elif in_grace_period:
            rospy.loginfo_throttle(1.0, f"🚀 宽限期内允许运动 - 剩余时间: {self.reset_grace_period - (current_time - self.path_updated_time):.1f}s")

        action = Twist()
        action.linear.x = speed
        action.angular.z = steer
        
        # 添加调试信息显示实际发布的速度
        rospy.loginfo_throttle(1.0, f"📤 发布速度命令 - 线速度: {speed:.3f}, 角速度: {steer:.3f}")

        return action

    def generate_dune_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.dune_points is None:
            return
        else:
            points = self.neupan_planner.dune_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_nrmp_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.nrmp_points is None:
            return
        else:
            points = self.neupan_planner.nrmp_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_robot_marker_msg(self):

        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.seq = 0
        marker.header.stamp = rospy.get_rostime()

        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = 1

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])

        return marker

    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw
