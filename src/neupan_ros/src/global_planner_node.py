#!/usr/bin/env python3

"""
Global Path Planner Node for NeuPAN Integration
Provides global path planning capabilities to work with NeuPAN local planner

Developed for NeuPAN ROS Integration
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import Header
import tf
from math import sqrt, atan2, cos, sin
import heapq
from typing import List, Tuple, Optional

class GlobalPathPlanner:
    def __init__(self):
        rospy.init_node('global_path_planner', anonymous=True)
        
        # Parameters
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.planner_type = rospy.get_param('~planner_type', 'astar')  # astar, dijkstra, rrt
        self.resolution = 0.05  # Will be updated from map
        self.inflation_radius = rospy.get_param('~inflation_radius', 0.3)
        self.replanning_enabled = rospy.get_param('~replanning_enabled', True)
        
        # State variables
        self.map_data = None
        self.map_info = None
        self.current_pose = None
        self.last_known_pose = None  # 备份最后已知位姿，用于多目标导航的鲁棒性
        self.goal_pose = None
        self.global_path = None
        self.path_blocked = False
        
        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.initial_path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)  # 重定位系统发布的里程计
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Global Path Planner initialized")
        
    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map data"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.resolution = msg.info.resolution
        rospy.loginfo("Map received: {}x{} resolution={}".format(
            msg.info.width, msg.info.height, self.resolution))
        
        # Inflate obstacles
        self.inflated_map = self.inflate_obstacles(self.map_data)
        
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current robot pose"""
        self.current_pose = msg.pose.pose
        
    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry"""
        self.current_pose = msg.pose.pose
        
    def goal_callback(self, msg: PoseStamped):
        """Handle new goal and plan path"""
        self.goal_pose = msg.pose
        rospy.loginfo("New goal received: ({:.2f}, {:.2f})".format(
            msg.pose.position.x, msg.pose.position.y))
        
        # 重置状态以确保能处理新目标
        self.path_blocked = False
        self.global_path = None  # 清除旧路径，强制重新规划
        
        # 调试信息：检查当前状态
        rospy.loginfo("Goal callback - current_pose: {}, map_data: {}, replanning_enabled: {}".format(
            self.current_pose is not None, 
            self.map_data is not None, 
            self.replanning_enabled))
        
        if self.current_pose is None:
            # Try to get current pose from TF
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    self.map_frame, 'base_link', rospy.Time(0))
                
                pose = PoseStamped()
                pose.pose.position.x = trans[0]
                pose.pose.position.y = trans[1]
                pose.pose.orientation.x = rot[0]
                pose.pose.orientation.y = rot[1]
                pose.pose.orientation.z = rot[2]
                pose.pose.orientation.w = rot[3]
                self.current_pose = pose.pose
                rospy.loginfo("Got current pose from TF: ({:.2f}, {:.2f})".format(trans[0], trans[1]))
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("Could not get robot pose from TF: {}".format(str(e)))
                # 如果无法从TF获取位姿，尝试使用最后已知的位姿
                if hasattr(self, 'last_known_pose') and self.last_known_pose is not None:
                    self.current_pose = self.last_known_pose
                    rospy.loginfo("Using last known pose as fallback")
                else:
                    rospy.logwarn("No current pose available, cannot plan path")
                    return
        
        # 立即尝试规划路径，而不是等待主循环
        if (self.current_pose is not None and 
            self.map_data is not None):
            rospy.loginfo("Attempting immediate path planning for new goal")
            success = self.plan_global_path()
            if success:
                rospy.loginfo("Immediate path planning successful")
            else:
                rospy.logwarn("Immediate path planning failed")
        else:
            rospy.logwarn("Cannot plan path immediately - missing requirements")
            rospy.logwarn("  current_pose: {}, map_data: {}".format(
                self.current_pose is not None, self.map_data is not None))
        
    def inflate_obstacles(self, map_data: np.ndarray) -> np.ndarray:
        """Inflate obstacles for safety margin"""
        inflated = map_data.copy()
        inflation_cells = int(self.inflation_radius / self.resolution)
        
        height, width = map_data.shape
        for i in range(height):
            for j in range(width):
                if map_data[i, j] > 50:  # Obstacle
                    # Inflate around this obstacle
                    for di in range(-inflation_cells, inflation_cells + 1):
                        for dj in range(-inflation_cells, inflation_cells + 1):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < height and 0 <= nj < width:
                                if sqrt(di*di + dj*dj) <= inflation_cells:
                                    inflated[ni, nj] = max(inflated[ni, nj], 99)
        
        return inflated
        
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map coordinates"""
        if self.map_info is None:
            return None, None
            
        mx = int((x - self.map_info.origin.position.x) / self.resolution)
        my = int((y - self.map_info.origin.position.y) / self.resolution)
        return mx, my
        
    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map coordinates to world coordinates"""
        if self.map_info is None:
            return None, None
            
        x = mx * self.resolution + self.map_info.origin.position.x
        y = my * self.resolution + self.map_info.origin.position.y
        return x, y
        
    def is_valid_cell(self, mx: int, my: int) -> bool:
        """Check if map cell is valid and free"""
        if self.inflated_map is None:
            return False
            
        height, width = self.inflated_map.shape
        if mx < 0 or mx >= width or my < 0 or my >= height:
            return False
            
        return self.inflated_map[my, mx] < 50  # Free space
        
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Heuristic function for A*"""
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    def astar_planning(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* path planning algorithm"""
        if not self.is_valid_cell(*start) or not self.is_valid_cell(*goal):
            rospy.logwarn("Start or goal position is not valid")
            return []
            
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 8-connected neighbors
        neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
                
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid_cell(*neighbor):
                    continue
                    
                # Cost for diagonal movement
                move_cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    
        rospy.logwarn("No path found!")
        return []
        
    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Simple path smoothing"""
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.line_of_sight(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed.append(path[i + 1])
                i += 1
                
        return smoothed
        
    def line_of_sight(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """Check if there's a clear line of sight between two points"""
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        x_step = 1 if x0 < x1 else -1
        y_step = 1 if y0 < y1 else -1
        
        error = dx - dy
        
        while True:
            if not self.is_valid_cell(x0, y0):
                return False
                
            if x0 == x1 and y0 == y1:
                break
                
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x0 += x_step
            if error2 < dx:
                error += dx
                y0 += y_step
                
        return True
        
    def plan_global_path(self):
        """Main path planning function"""
        if self.map_data is None or self.current_pose is None or self.goal_pose is None:
            rospy.logwarn("Missing required data for path planning")
            rospy.logwarn("  map_data: {}, current_pose: {}, goal_pose: {}".format(
                self.map_data is not None, self.current_pose is not None, self.goal_pose is not None))
            return False
            
        # Convert poses to map coordinates
        start_mx, start_my = self.world_to_map(
            self.current_pose.position.x, self.current_pose.position.y)
        goal_mx, goal_my = self.world_to_map(
            self.goal_pose.position.x, self.goal_pose.position.y)
            
        if start_mx is None or goal_mx is None:
            rospy.logwarn("Could not convert poses to map coordinates")
            return False
            
        rospy.loginfo("🗺️ Planning path from ({},{}) to ({},{}) in map coordinates".format(
            start_mx, start_my, goal_mx, goal_my))
            
        # Plan path using selected algorithm
        if self.planner_type == 'astar':
            path_map = self.astar_planning((start_mx, start_my), (goal_mx, goal_my))
        else:
            rospy.logwarn("Unsupported planner type: {}".format(self.planner_type))
            return False
            
        if not path_map:
            rospy.logwarn("❌ Path planning failed!")
            return False
            
        rospy.loginfo("✅ Path planning successful with {} waypoints".format(len(path_map)))
            
        # Smooth the path
        smoothed_path = self.smooth_path(path_map)
        
        # Convert back to world coordinates and create ROS path
        self.create_ros_path(smoothed_path)
        return True
        
    def create_ros_path(self, path_map: List[Tuple[int, int]]):
        """Convert map path to ROS Path message"""
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()
        
        for mx, my in path_map:
            x, y = self.map_to_world(mx, my)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            
            # Calculate orientation based on next waypoint
            if len(path_msg.poses) > 0:
                prev_pose = path_msg.poses[-1]
                dx = x - prev_pose.pose.position.x
                dy = y - prev_pose.pose.position.y
                yaw = atan2(dy, dx)
                
                pose_stamped.pose.orientation.z = sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = cos(yaw / 2.0)
            
            path_msg.poses.append(pose_stamped)
        
        # Publish paths
        self.path_pub.publish(path_msg)
        self.initial_path_pub.publish(path_msg)  # For NeuPAN
        
        self.global_path = path_msg
        rospy.loginfo("Global path published with {} waypoints".format(len(path_msg.poses)))
        
    def run(self):
        """Main loop with dynamic replanning"""
        rate = rospy.Rate(5)  # 提高到5Hz，更频繁地检查路径状态
        
        while not rospy.is_shutdown():
            # 如果有目标和地图，且启用重规划
            if (self.goal_pose is not None and 
                self.map_data is not None and 
                self.current_pose is not None and 
                self.replanning_enabled):
                
                # 检查是否需要重新规划
                need_replan = False
                
                # 情况1：还没有路径
                if self.global_path is None:
                    need_replan = True
                    rospy.loginfo("No existing path, planning initial path")
                
                # 情况2：路径被阻挡
                elif self.path_blocked:
                    need_replan = True
                    rospy.loginfo("Path blocked, replanning")
                
                # 情况3：机器人偏离路径太远（可选）
                elif self.is_robot_far_from_path():
                    need_replan = True
                    rospy.loginfo("Robot too far from path, replanning")
                
                if need_replan:
                    success = self.plan_global_path()
                    if success:
                        rospy.loginfo("Replanning successful")
                    else:
                        rospy.logwarn("Replanning failed, keeping old path")
                
                # 重新发布现有路径（为NeuPAN提供持续的路径信息）
                elif self.global_path is not None:
                    self.initial_path_pub.publish(self.global_path)
            
            rate.sleep()
    
    def is_robot_far_from_path(self):
        """检查机器人是否偏离路径太远"""
        if self.global_path is None or self.current_pose is None:
            return False
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # 找到路径上最近的点
        min_distance = float('inf')
        for pose in self.global_path.poses:
            dx = robot_x - pose.pose.position.x
            dy = robot_y - pose.pose.position.y
            distance = (dx**2 + dy**2)**0.5
            min_distance = min(min_distance, distance)
        
        # 如果距离路径超过0.5米，认为偏离太远
        return min_distance > 0.5

if __name__ == '__main__':
    try:
        planner = GlobalPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
