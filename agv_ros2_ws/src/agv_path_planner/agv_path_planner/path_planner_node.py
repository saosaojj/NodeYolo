# AGV路径规划节点，基于A*算法在栅格地图上进行路径规划
# 支持激光扫描更新地图、路径平滑、Dijkstra回退规划等功能
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import heapq
import math
from typing import Tuple, List, Optional, Set

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs

from .smooth_astar import SmoothAStar


# A*搜索用的栅格节点类，存储位置、代价和父节点信息
class GridNode:
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: 'GridNode' = None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    # 比较运算符，用于优先队列排序（f值小的优先）
    def __lt__(self, other):
        return self.f < other.f

    # 相等判断，基于坐标
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    # 哈希值，用于集合去重
    def __hash__(self):
        return hash((self.x, self.y))


# 路径规划节点类，管理栅格地图、执行路径搜索和发布规划结果
class PathPlannerNode(Node):
    # 初始化路径规划节点，声明参数并创建通信接口
    def __init__(self):
        super().__init__('path_planner_node')
        
        # 声明规划参数：地图分辨率、尺寸、机器人半径、安全距离等
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 20.0)
        self.declare_parameter('map_height', 20.0)
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('planning_timeout', 5.0)
        self.declare_parameter('max_iterations', 100000)
        self.declare_parameter('heuristic_weight', 1.0)
        
        # 读取参数值
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        
        # 计算栅格地图尺寸
        self.grid_width = int(self.map_width / self.map_resolution)
        self.grid_height = int(self.map_height / self.map_resolution)
        
        # 初始化占据栅格地图（0=空闲，100=占据）
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        # 当前规划路径和路径代价
        self.current_path = None
        self.path_cost_value = 0.0
        
        # 创建平滑A*规划器实例
        self.smooth_astar = SmoothAStar(
            self.occupancy_grid,
            self.grid_width,
            self.grid_height,
            self.heuristic_weight
        )
        
        # QoS配置，使用BEST_EFFORT策略以适应传感器数据
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布路径可视化栅格地图
        self.path_visualization_pub = self.create_publisher(
            nav_msgs.OccupancyGrid,
            'path_visualization',
            10
        )
        # 发布规划路径
        self.planned_path_pub = self.create_publisher(
            nav_msgs.Path,
            'planned_path',
            10
        )
        # 发布路径代价
        self.path_cost_pub = self.create_publisher(
            std_msgs.Float32,
            'path_cost',
            10
        )
        
        # 订阅激光扫描话题，用于更新地图障碍物
        self.laser_scan_sub = self.create_subscription(
            sensor_msgs.LaserScan,
            'laser_scan',
            self.laser_scan_callback,
            qos_profile
        )
        # 订阅目标位姿话题，触发路径规划
        self.target_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'target_pose',
            self.target_pose_callback,
            qos_profile
        )
        # 订阅地图更新话题，接收外部地图数据
        self.map_update_sub = self.create_subscription(
            nav_msgs.OccupancyGrid,
            'map_update',
            self.map_update_callback,
            qos_profile
        )
        
        # 创建路径规划触发服务
        self.path_plan_service = self.create_service(
            std_srvs.Trigger,
            '/path_plan',
            self.path_plan_callback
        )
        # 创建路径清除服务
        self.path_clear_service = self.create_service(
            std_srvs.Trigger,
            '/path_clear',
            self.path_clear_callback
        )
        # 创建地图重置服务
        self.map_reset_service = self.create_service(
            std_srvs.Trigger,
            '/map_reset',
            self.map_reset_callback
        )
        
        self.get_logger().info('Path Planner Node initialized')
        self.get_logger().info(f'Grid size: {self.grid_width}x{self.grid_height}')

    # 更新平滑A*规划器的栅格数据
    def build_grid(self):
        self.smooth_astar.update_grid(self.occupancy_grid)

    # 将世界坐标转换为栅格坐标
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x + self.map_width / 2) / self.map_resolution)
        gy = int((y + self.map_height / 2) / self.map_resolution)
        gx = max(0, min(gx, self.grid_width - 1))
        gy = max(0, min(gy, self.grid_height - 1))
        return gx, gy

    # 将栅格坐标转换为世界坐标
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.map_resolution - self.map_width / 2
        y = gy * self.map_resolution - self.map_height / 2
        return x, y

    # 启发式函数，计算两节点间的欧几里得距离（带权重）
    def heuristic(self, node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        dx = abs(node1[0] - node2[0])
        dy = abs(node1[1] - node2[1])
        return math.sqrt(dx * dx + dy * dy) * self.heuristic_weight

    # 获取节点的8邻域邻居，过滤障碍物和对角线穿越障碍的情况
    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        # 8方向：上、下、左、右及四个对角线
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                # 检查邻居是否为空闲格子（占据概率<50）
                if self.occupancy_grid[ny, nx] < 50:
                    # 对角线移动时检查相邻格子是否也被占据，防止穿越障碍物角落
                    if dx != 0 and dy != 0:
                        if self.occupancy_grid[node[1], node[0] + dx] >= 50 or \
                           self.occupancy_grid[node[1] + dy, node[0]] >= 50:
                            continue
                    neighbors.append((nx, ny))
        return neighbors

    # A*路径搜索算法，在栅格地图上寻找从起点到终点的最短路径
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        # 目标点在障碍物中则无法到达
        if self.occupancy_grid[goal[1], goal[0]] >= 50:
            return None
        
        open_set = []
        start_node = GridNode(start[0], start[1], 0, self.heuristic(start, goal))
        heapq.heappush(open_set, start_node)
        
        closed_set = set()
        g_scores = {start: 0}
        
        iterations = 0
        
        while open_set and iterations < self.max_iterations:
            iterations += 1
            
            current = heapq.heappop(open_set)
            
            # 到达目标，回溯构建路径
            if (current.x, current.y) == goal:
                path = []
                node = current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                return path[::-1]
            
            closed_set.add((current.x, current.y))
            
            # 扩展邻居节点
            for neighbor in self.get_neighbors((current.x, current.y)):
                if neighbor in closed_set:
                    continue
                
                # 计算移动代价（对角线为√2，直线为1）
                move_cost = math.sqrt(
                    (neighbor[0] - current.x) ** 2 + 
                    (neighbor[1] - current.y) ** 2
                )
                tentative_g = g_scores[(current.x, current.y)] + move_cost
                
                # 找到更优路径或首次访问该节点
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    h = self.heuristic(neighbor, goal)
                    neighbor_node = GridNode(
                        neighbor[0], neighbor[1], 
                        tentative_g, h, current
                    )
                    heapq.heappush(open_set, neighbor_node)
        
        return None

    # 路径平滑，通过视线检测移除不必要的中间路径点
    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        current_idx = 0
        
        # 贪心策略：从当前点找到最远的可直接到达的路径点
        while current_idx < len(path) - 1:
            farthest = current_idx + 1
            
            for check_idx in range(len(path) - 1, current_idx, -1):
                if self.check_line_of_sight(path[current_idx], path[check_idx]):
                    farthest = check_idx
                    break
            
            smoothed.append(path[farthest])
            current_idx = farthest
        
        return smoothed

    # 视线检测，使用Bresenham算法检查两点之间是否有障碍物
    def check_line_of_sight(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
        x0, y0 = p1
        x1, y1 = p2
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # 遇到障碍物则视线不通
            if self.occupancy_grid[y, x] >= 50:
                return False
            
            if x == x1 and y == y1:
                return True
            
            # Bresenham算法步进
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # Dijkstra回退搜索算法，当A*失败时使用无启发式的Dijkstra搜索
    def dijkstra_fallback(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        if self.occupancy_grid[goal[1], goal[0]] >= 50:
            return None
        
        open_set = [(0, start)]
        closed_set = set()
        g_scores = {start: 0}
        parents = {start: None}
        
        iterations = 0
        
        while open_set and iterations < self.max_iterations:
            iterations += 1
            
            current_g, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            # 到达目标，回溯构建路径
            if current == goal:
                path = []
                node = current
                while node:
                    path.append(node)
                    node = parents.get(node)
                return path[::-1]
            
            closed_set.add(current)
            
            # 扩展邻居节点（统一移动代价为1）
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                move_cost = 1.0
                tentative_g = g_scores.get(current, float('inf')) + move_cost
                
                if tentative_g < g_scores.get(neighbor, float('inf')):
                    g_scores[neighbor] = tentative_g
                    parents[neighbor] = current
                    heapq.heappush(open_set, (tentative_g, neighbor))
        
        return None

    # 路径规划主函数，从世界坐标起点到终点执行完整规划流程
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        # 世界坐标转栅格坐标
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        # 检查目标是否越界
        if not (0 <= goal_grid[0] < self.grid_width and 0 <= goal_grid[1] < self.grid_height):
            self.get_logger().error('Goal position out of bounds')
            return None
        
        # 如果目标在障碍物中，搜索附近可到达的位置
        if self.occupancy_grid[goal_grid[1], goal_grid[0]] >= 50:
            self.get_logger().warn('Goal is in obstacle, trying nearby positions')
            for radius in range(1, 5):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if abs(dx) == radius or abs(dy) == radius:
                            nx, ny = goal_grid[0] + dx, goal_grid[1] + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                if self.occupancy_grid[ny, nx] < 50:
                                    goal_grid = (nx, ny)
                                    break
                        if abs(dx) == radius or abs(dy) == radius:
                            if self.occupancy_grid[goal_grid[1], goal_grid[0]] < 50:
                                break
                    else:
                        continue
                    break
                else:
                    continue
                break
        
        self.get_logger().info(f'Planning from {start_grid} to {goal_grid}')
        
        # 首先尝试A*搜索
        path = self.astar(start_grid, goal_grid)
        
        # A*失败时回退到Dijkstra搜索
        if path is None:
            self.get_logger().warn('A* failed, trying Dijkstra fallback')
            path = self.dijkstra_fallback(start_grid, goal_grid)
        
        if path is None:
            self.get_logger().error('No path found')
            return None
        
        # 对路径进行平滑处理
        path = self.smooth_path(path)
        
        # 将栅格坐标转换回世界坐标
        world_path = [self.grid_to_world(p[0], p[1]) for p in path]
        
        # 计算路径总代价
        total_cost = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_cost += math.sqrt(dx * dx + dy * dy) * self.map_resolution
        
        self.path_cost_value = total_cost
        
        return world_path

    # 根据激光扫描数据更新地图障碍物
    def update_map_from_laser(self, scan: sensor_msgs.LaserScan):
        angle = scan.angle_min
        for i, range_val in enumerate(scan.ranges):
            if not math.isinf(range_val) and not math.isnan(range_val):
                if range_val < scan.range_max and range_val > scan.range_min:
                    # 计算障碍物位置（包含机器人半径和安全距离的膨胀）
                    obs_distance = range_val + self.robot_radius + self.safe_distance
                    obs_x = obs_distance * math.cos(angle)
                    obs_y = obs_distance * math.sin(angle)
                    
                    gx, gy = self.world_to_grid(obs_x, obs_y)
                    
                    # 在障碍物周围标记膨胀区域
                    radius_in_cells = int((self.robot_radius + self.safe_distance) / self.map_resolution)
                    
                    for dx in range(-radius_in_cells, radius_in_cells + 1):
                        for dy in range(-radius_in_cells, radius_in_cells + 1):
                            nx, ny = gx + dx, gy + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                dist = math.sqrt(dx * dx + dy * dy) * self.map_resolution
                                if dist <= self.robot_radius + self.safe_distance:
                                    self.occupancy_grid[ny, nx] = 100
        
        self.build_grid()

    # 在指定位置添加圆形障碍物
    def add_obstacle(self, x: float, y: float, radius: float):
        gx, gy = self.world_to_grid(x, y)
        radius_in_cells = int(radius / self.map_resolution)
        
        for dx in range(-radius_in_cells, radius_in_cells + 1):
            for dy in range(-radius_in_cells, radius_in_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                    if math.sqrt(dx * dx + dy * dy) * self.map_resolution <= radius:
                        self.occupancy_grid[ny, nx] = 100
        
        self.build_grid()

    # 清除当前路径和路径代价
    def clear_path(self):
        self.current_path = None
        self.path_cost_value = 0.0

    # 发布路径可视化栅格地图（在地图上标记路径点）
    def publish_path_visualization(self):
        grid_msg = nav_msgs.OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = -self.map_width / 2
        grid_msg.info.origin.position.y = -self.map_height / 2
        
        # 复制地图并在路径点位置标记为50（灰色）
        visualization_grid = self.occupancy_grid.copy()
        
        if self.current_path:
            for point in self.current_path:
                gx, gy = self.world_to_grid(point[0], point[1])
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    visualization_grid[gy, gx] = 50
        
        grid_msg.data = visualization_grid.flatten().tolist()
        self.path_visualization_pub.publish(grid_msg)

    # 发布规划路径和路径代价
    def publish_planned_path(self):
        if self.current_path is None:
            return
        
        # 构建Path消息
        path_msg = nav_msgs.Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in self.current_path:
            pose = geometry_msgs.PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.planned_path_pub.publish(path_msg)
        
        # 发布路径代价
        cost_msg = std_msgs.Float32()
        cost_msg.data = self.path_cost_value
        self.path_cost_pub.publish(cost_msg)

    # 激光扫描回调，根据扫描数据更新地图
    def laser_scan_callback(self, msg: sensor_msgs.LaserScan):
        self.update_map_from_laser(msg)

    # 目标位姿回调，接收目标后触发路径规划
    def target_pose_callback(self, msg: geometry_msgs.PoseStamped):
        start = (0.0, 0.0)
        goal = (msg.pose.position.x, msg.pose.position.y)
        
        self.get_logger().info(f'Received target: {goal}')
        
        path = self.plan_path(start, goal)
        
        if path:
            self.current_path = path
            self.get_logger().info(f'Path planned with {len(path)} waypoints')
        else:
            self.current_path = None
            self.get_logger().warn('Failed to plan path')

    # 地图更新回调，接收外部地图数据替换当前地图
    def map_update_callback(self, msg: nav_msgs.OccupancyGrid):
        if msg.info.width != self.grid_width or msg.info.height != self.grid_height:
            self.get_logger().warn('Map update dimensions do not match, ignoring')
            return
        
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.build_grid()
        self.get_logger().info('Map updated from external source')

    # 路径规划服务回调，返回当前路径状态
    def path_plan_callback(self, request, response):
        if self.current_path is None:
            response.success = False
            response.message = 'No path available'
        else:
            response.success = True
            response.message = f'Path exists with {len(self.current_path)} waypoints'
        return response

    # 路径清除服务回调
    def path_clear_callback(self, request, response):
        self.clear_path()
        response.success = True
        response.message = 'Path cleared'
        return response

    # 地图重置服务回调，清空地图和路径
    def map_reset_callback(self, request, response):
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.clear_path()
        self.build_grid()
        response.success = True
        response.message = 'Map reset to empty'
        return response


# 节点主入口函数，使用spin_once循环以支持可视化发布
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.publish_path_visualization()
            node.publish_planned_path()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
