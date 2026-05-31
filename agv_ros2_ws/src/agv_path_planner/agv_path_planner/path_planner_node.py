import rclpy
from rclpy.node import Node as ROSNode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import heapq
import math
import time
import json
from typing import Tuple, List, Optional

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs

from agv_interfaces.msg import AGVStatus

from .smooth_astar import SmoothAStar


class _GridNode:
    """A*搜索用的网格节点"""
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: '_GridNode' = None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


class PathPlannerNode(ROSNode):
    def __init__(self):
        super().__init__('path_planner_node')

        # 声明原有参数
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 20.0)
        self.declare_parameter('map_height', 20.0)
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('planning_timeout', 5.0)
        self.declare_parameter('max_iterations', 100000)
        self.declare_parameter('heuristic_weight', 1.0)

        # 声明新增参数：代价地图膨胀
        self.declare_parameter('inflation_radius', 0.5)
        self.declare_parameter('cost_scaling_factor', 10.0)

        # 声明新增参数：动态重规划
        self.declare_parameter('replan_interval', 2.0)
        self.declare_parameter('robot_moved_threshold', 1.0)

        # 声明新增参数：B样条平滑
        self.declare_parameter('bspline_smoothing', True)
        self.declare_parameter('path_sample_interval', 0.1)

        # 声明新增参数：路径验证
        self.declare_parameter('max_path_length_ratio', 3.0)
        self.declare_parameter('max_turn_angle', 1.57)

        # 读取原有参数
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value

        # 读取新增参数
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.cost_scaling_factor = self.get_parameter('cost_scaling_factor').value
        self.replan_interval = self.get_parameter('replan_interval').value
        self.robot_moved_threshold = self.get_parameter('robot_moved_threshold').value
        self.bspline_smoothing = self.get_parameter('bspline_smoothing').value
        self.path_sample_interval = self.get_parameter('path_sample_interval').value
        self.max_path_length_ratio = self.get_parameter('max_path_length_ratio').value
        self.max_turn_angle = self.get_parameter('max_turn_angle').value

        self.grid_width = int(self.map_width / self.map_resolution)
        self.grid_height = int(self.map_height / self.map_resolution)

        # 占用栅格和代价地图
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.cost_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float64)

        # 路径状态
        self.current_path = None
        self.current_path_orientations = []
        self.path_cost_value = 0.0
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0

        # 重规划追踪
        self._last_plan_start = None
        self._last_plan_goal = None
        self._last_plan_time = 0.0

        self.smooth_astar = SmoothAStar(
            self.occupancy_grid,
            self.grid_width,
            self.grid_height,
            self.heuristic_weight
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 发布器
        self.path_visualization_pub = self.create_publisher(
            nav_msgs.OccupancyGrid,
            'path_visualization',
            10
        )
        self.planned_path_pub = self.create_publisher(
            nav_msgs.Path,
            'planned_path',
            10
        )
        self.path_cost_pub = self.create_publisher(
            std_msgs.Float32,
            'path_cost',
            10
        )
        self.path_metadata_pub = self.create_publisher(
            std_msgs.String,
            'path_metadata',
            10
        )

        # 订阅器
        self.laser_scan_sub = self.create_subscription(
            sensor_msgs.LaserScan,
            'laser_scan',
            self.laser_scan_callback,
            qos_profile
        )
        self.target_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'target_pose',
            self.target_pose_callback,
            qos_profile
        )
        self.map_update_sub = self.create_subscription(
            nav_msgs.OccupancyGrid,
            'map_update',
            self.map_update_callback,
            qos_profile
        )
        self.agv_status_sub = self.create_subscription(
            AGVStatus,
            'agv_status',
            self.agv_status_callback,
            10
        )

        # 服务
        self.path_plan_service = self.create_service(
            std_srvs.Trigger,
            '/path_plan',
            self.path_plan_callback
        )
        self.path_clear_service = self.create_service(
            std_srvs.Trigger,
            '/path_clear',
            self.path_clear_callback
        )
        self.map_reset_service = self.create_service(
            std_srvs.Trigger,
            '/map_reset',
            self.map_reset_callback
        )

        # 定时器：动态重规划检查
        self.replan_timer = self.create_timer(
            self.replan_interval,
            self.replan_check_callback
        )

        # 定时器：路径可视化（1.0 Hz）
        self.visualization_timer = self.create_timer(
            1.0,
            self.publish_path_visualization
        )

        # 定时器：路径发布（2.0 Hz）
        self.path_publish_timer = self.create_timer(
            0.5,
            self.publish_planned_path
        )

        self.get_logger().info('路径规划节点已初始化')
        self.get_logger().info(f'栅格尺寸: {self.grid_width}x{self.grid_height}')
        self.get_logger().info(f'膨胀半径: {self.inflation_radius}, 代价缩放因子: {self.cost_scaling_factor}')
        self.get_logger().info(f'重规划间隔: {self.replan_interval}s, 机器人移动阈值: {self.robot_moved_threshold}m')

    def build_grid(self):
        self.smooth_astar.update_grid(self.occupancy_grid)

    def inflate_obstacles(self):
        """根据占用栅格生成膨胀代价地图"""
        self.cost_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float64)

        inflation_cells = int(self.inflation_radius / self.map_resolution)
        robot_cells = int(self.robot_radius / self.map_resolution)

        obstacle_positions = np.argwhere(self.occupancy_grid >= 50)

        for obs_y, obs_x in obstacle_positions:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx, ny = obs_x + dx, obs_y + dy
                    if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                        dist_cells = math.sqrt(dx * dx + dy * dy)
                        dist_meters = dist_cells * self.map_resolution

                        if dist_meters <= self.robot_radius:
                            self.cost_map[ny, nx] = 253.0
                        elif dist_meters <= self.inflation_radius:
                            cost = 253.0 * math.exp(
                                -self.cost_scaling_factor * (dist_meters - self.robot_radius)
                            )
                            if cost > self.cost_map[ny, nx]:
                                self.cost_map[ny, nx] = cost

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x + self.map_width / 2) / self.map_resolution)
        gy = int((y + self.map_height / 2) / self.map_resolution)
        gx = max(0, min(gx, self.grid_width - 1))
        gy = max(0, min(gy, self.grid_height - 1))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.map_resolution - self.map_width / 2
        y = gy * self.map_resolution - self.map_height / 2
        return x, y

    def heuristic(self, node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        dx = abs(node1[0] - node2[0])
        dy = abs(node1[1] - node2[1])
        return math.sqrt(dx * dx + dy * dy) * self.heuristic_weight

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                if self.occupancy_grid[ny, nx] < 50:
                    if dx != 0 and dy != 0:
                        if self.occupancy_grid[node[1], node[0] + dx] >= 50 or \
                           self.occupancy_grid[node[1] + dy, node[0]] >= 50:
                            continue
                    neighbors.append((nx, ny))
        return neighbors

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """带代价地图的A*搜索"""
        if self.occupancy_grid[goal[1], goal[0]] >= 50:
            return None

        open_set = []
        start_node = _GridNode(start[0], start[1], 0, self.heuristic(start, goal))
        heapq.heappush(open_set, start_node)

        closed_set = set()
        g_scores = {start: 0}

        iterations = 0

        while open_set and iterations < self.max_iterations:
            iterations += 1

            current = heapq.heappop(open_set)

            if (current.x, current.y) == goal:
                path = []
                node = current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                return path[::-1]

            closed_set.add((current.x, current.y))

            for neighbor in self.get_neighbors((current.x, current.y)):
                if neighbor in closed_set:
                    continue

                move_cost = math.sqrt(
                    (neighbor[0] - current.x) ** 2 +
                    (neighbor[1] - current.y) ** 2
                )

                # 加入代价地图的代价值
                cost_map_value = self.cost_map[neighbor[1], neighbor[0]] if \
                    0 <= neighbor[1] < self.grid_height and 0 <= neighbor[0] < self.grid_width else 0.0
                move_cost += cost_map_value / 253.0

                tentative_g = g_scores[(current.x, current.y)] + move_cost

                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    h = self.heuristic(neighbor, goal)
                    neighbor_node = _GridNode(
                        neighbor[0], neighbor[1],
                        tentative_g, h, current
                    )
                    heapq.heappush(open_set, neighbor_node)

        return None

    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """视线平滑：去除不必要的中间路径点"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            farthest = current_idx + 1

            for check_idx in range(len(path) - 1, current_idx, -1):
                if self.check_line_of_sight(path[current_idx], path[check_idx]):
                    farthest = check_idx
                    break

            smoothed.append(path[farthest])
            current_idx = farthest

        return smoothed

    def bspline_smooth(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """使用B样条对路径进行进一步平滑"""
        if not self.bspline_smoothing:
            return path

        if len(path) < 4:
            return path

        num_samples = max(len(path), int(
            self.smooth_astar.compute_path_length(path, self.map_resolution) /
            self.path_sample_interval
        ))
        num_samples = min(num_samples, 1000)

        return self.smooth_astar.bspline_smooth(path, num_samples=num_samples)

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
            if self.occupancy_grid[y, x] >= 50:
                return False

            if x == x1 and y == y1:
                return True

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

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

            if current == goal:
                path = []
                node = current
                while node:
                    path.append(node)
                    node = parents.get(node)
                return path[::-1]

            closed_set.add(current)

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

    def validate_path(self, path: List[Tuple[int, int]], start_world: Tuple[float, float],
                      goal_world: Tuple[float, float]) -> bool:
        """验证路径有效性：障碍物碰撞、路径长度比、急转弯"""
        if not path:
            return False

        # 检查路径点是否在障碍物内
        for node in path:
            nx, ny = node
            if nx < 0 or nx >= self.grid_width or ny < 0 or ny >= self.grid_height:
                self.get_logger().warn('路径验证失败：路径点超出边界')
                return False
            if self.occupancy_grid[ny, nx] >= 50:
                self.get_logger().warn('路径验证失败：路径点位于障碍物内')
                return False

        # 检查路径长度比
        path_length = self.smooth_astar.compute_path_length(path, self.map_resolution)
        straight_dist = math.sqrt(
            (goal_world[0] - start_world[0]) ** 2 +
            (goal_world[1] - start_world[1]) ** 2
        )
        if straight_dist > 0 and path_length / straight_dist > self.max_path_length_ratio:
            self.get_logger().warn(
                f'路径验证失败：路径长度比 {path_length / straight_dist:.2f} 超过阈值 {self.max_path_length_ratio}'
            )
            return False

        # 检查急转弯
        if not self.smooth_astar.validate_path(path, self.max_turn_angle):
            self.get_logger().warn('路径验证失败：存在超过阈值的急转弯')
            return False

        return True

    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])

        if not (0 <= goal_grid[0] < self.grid_width and 0 <= goal_grid[1] < self.grid_height):
            self.get_logger().error('目标位置超出边界')
            return None

        if self.occupancy_grid[goal_grid[1], goal_grid[0]] >= 50:
            self.get_logger().warn('目标位于障碍物内，尝试附近位置')
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

        # 更新代价地图
        self.inflate_obstacles()

        self.get_logger().info(f'开始规划：{start_grid} -> {goal_grid}')

        plan_start_time = time.time()

        path = self.astar(start_grid, goal_grid)

        if path is None:
            self.get_logger().warn('A*规划失败，尝试Dijkstra后备方案')
            path = self.dijkstra_fallback(start_grid, goal_grid)

        if path is None:
            self.get_logger().error('未找到路径')
            return None

        # 视线平滑
        path = self.smooth_path(path)

        # B样条平滑
        path = self.bspline_smooth(path)

        # 路径验证
        if not self.validate_path(path, start, goal):
            self.get_logger().warn('路径验证失败，尝试重新规划')
            path = self.astar(start_grid, goal_grid)
            if path is not None:
                path = self.smooth_path(path)
                if not self.validate_path(path, start, goal):
                    self.get_logger().error('重新规划后路径仍然无效')
                    return None
            else:
                return None

        # 计算路径航向角
        self.current_path_orientations = self.smooth_astar.compute_path_orientations(path)

        world_path = [self.grid_to_world(p[0], p[1]) for p in path]

        total_cost = self.smooth_astar.compute_path_length(path, self.map_resolution)
        self.path_cost_value = total_cost

        planning_time_ms = (time.time() - plan_start_time) * 1000.0

        # 记录上次规划信息
        self._last_plan_start = start
        self._last_plan_goal = goal
        self._last_plan_time = time.time()

        # 发布路径元数据
        self.publish_path_metadata(
            total_length=total_cost,
            num_waypoints=len(world_path),
            planning_time_ms=planning_time_ms,
            algorithm_used='A*',
            is_valid=True
        )

        return world_path

    def replan_check_callback(self):
        """定时检查是否需要重新规划路径"""
        if self.current_path is None:
            return

        if self._last_plan_start is None:
            return

        # 检查机器人是否偏离上次规划起点过远
        dx = self.current_robot_x - self._last_plan_start[0]
        dy = self.current_robot_y - self._last_plan_start[1]
        dist_from_start = math.sqrt(dx * dx + dy * dy)

        need_replan = False

        if dist_from_start > self.robot_moved_threshold:
            self.get_logger().info(
                f'机器人已偏离起点 {dist_from_start:.2f}m，触发重规划'
            )
            need_replan = True

        # 检查当前路径上是否有新障碍物
        if not need_replan and self.current_path:
            for point in self.current_path:
                gx, gy = self.world_to_grid(point[0], point[1])
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    if self.occupancy_grid[gy, gx] >= 50:
                        self.get_logger().info('当前路径上检测到新障碍物，触发重规划')
                        need_replan = True
                        break

        if need_replan and self._last_plan_goal is not None:
            start = (self.current_robot_x, self.current_robot_y)
            goal = self._last_plan_goal

            # 避免无意义的重规划：检查起点和目标是否与上次相同
            if self._last_plan_start is not None:
                start_dx = start[0] - self._last_plan_start[0]
                start_dy = start[1] - self._last_plan_start[1]
                if math.sqrt(start_dx ** 2 + start_dy ** 2) < 0.1:
                    return

            self.get_logger().info('执行动态重规划...')
            path = self.plan_path(start, goal)

            if path:
                self.current_path = path
                self.get_logger().info(f'重规划完成，共 {len(path)} 个路径点')
            else:
                self.get_logger().warn('重规划失败，保留原路径')

    def update_map_from_laser(self, scan: sensor_msgs.LaserScan):
        angle = scan.angle_min
        for i, range_val in enumerate(scan.ranges):
            if not math.isinf(range_val) and not math.isnan(range_val):
                if range_val < scan.range_max and range_val > scan.range_min:
                    obs_distance = range_val + self.robot_radius + self.safe_distance
                    obs_x = obs_distance * math.cos(angle)
                    obs_y = obs_distance * math.sin(angle)

                    gx, gy = self.world_to_grid(obs_x, obs_y)

                    radius_in_cells = int((self.robot_radius + self.safe_distance) / self.map_resolution)

                    for dx in range(-radius_in_cells, radius_in_cells + 1):
                        for dy in range(-radius_in_cells, radius_in_cells + 1):
                            nx, ny = gx + dx, gy + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                dist = math.sqrt(dx * dx + dy * dy) * self.map_resolution
                                if dist <= self.robot_radius + self.safe_distance:
                                    self.occupancy_grid[ny, nx] = 100

            angle += scan.angle_increment

        self.build_grid()

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

    def clear_path(self):
        self.current_path = None
        self.current_path_orientations = []
        self.path_cost_value = 0.0
        self._last_plan_start = None
        self._last_plan_goal = None

    def publish_path_visualization(self):
        grid_msg = nav_msgs.OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = -self.map_width / 2
        grid_msg.info.origin.position.y = -self.map_height / 2

        visualization_grid = self.occupancy_grid.copy()

        if self.current_path:
            for point in self.current_path:
                gx, gy = self.world_to_grid(point[0], point[1])
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    visualization_grid[gy, gx] = 50

        grid_msg.data = visualization_grid.flatten().tolist()
        self.path_visualization_pub.publish(grid_msg)

    def publish_planned_path(self):
        if self.current_path is None:
            return

        path_msg = nav_msgs.Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i, point in enumerate(self.current_path):
            pose = geometry_msgs.PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0

            # 设置航向角四元数
            if i < len(self.current_path_orientations):
                yaw = self.current_path_orientations[i]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.planned_path_pub.publish(path_msg)

        cost_msg = std_msgs.Float32()
        cost_msg.data = self.path_cost_value
        self.path_cost_pub.publish(cost_msg)

    def publish_path_metadata(self, total_length: float, num_waypoints: int,
                              planning_time_ms: float, algorithm_used: str, is_valid: bool):
        """发布路径元数据（JSON格式）"""
        metadata = {
            'total_length': round(total_length, 4),
            'num_waypoints': num_waypoints,
            'planning_time_ms': round(planning_time_ms, 2),
            'algorithm_used': algorithm_used,
            'is_valid': is_valid
        }

        msg = std_msgs.String()
        msg.data = json.dumps(metadata, ensure_ascii=False)
        self.path_metadata_pub.publish(msg)

    def laser_scan_callback(self, msg: sensor_msgs.LaserScan):
        self.update_map_from_laser(msg)

    def target_pose_callback(self, msg: geometry_msgs.PoseStamped):
        start = (self.current_robot_x, self.current_robot_y)
        goal = (msg.pose.position.x, msg.pose.position.y)

        self.get_logger().info(f'收到目标: {goal}，起点: ({self.current_robot_x:.2f}, {self.current_robot_y:.2f})')

        path = self.plan_path(start, goal)

        if path:
            self.current_path = path
            self.get_logger().info(f'路径规划完成，共 {len(path)} 个路径点')
        else:
            self.current_path = None
            self.get_logger().warn('路径规划失败')

    def agv_status_callback(self, msg):
        self.current_robot_x = msg.x
        self.current_robot_y = msg.y

    def map_update_callback(self, msg: nav_msgs.OccupancyGrid):
        if msg.info.width != self.grid_width or msg.info.height != self.grid_height:
            self.get_logger().warn('地图更新尺寸不匹配，忽略')
            return

        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.build_grid()
        self.get_logger().info('从外部源更新地图')

    def path_plan_callback(self, request, response):
        if self.current_path is None:
            response.success = False
            response.message = 'No path available'
        else:
            response.success = True
            response.message = f'Path exists with {len(self.current_path)} waypoints'
        return response

    def path_clear_callback(self, request, response):
        self.clear_path()
        response.success = True
        response.message = 'Path cleared'
        return response

    def map_reset_callback(self, request, response):
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.cost_map = np.zeros((self.grid_height, self.grid_width), dtype=np.float64)
        self.clear_path()
        self.build_grid()
        response.success = True
        response.message = 'Map reset to empty'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
