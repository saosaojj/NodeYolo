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


class Node:
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: 'Node' = None):
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


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 20.0)
        self.declare_parameter('map_height', 20.0)
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('planning_timeout', 5.0)
        self.declare_parameter('max_iterations', 100000)
        self.declare_parameter('heuristic_weight', 1.0)
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        
        self.grid_width = int(self.map_width / self.map_resolution)
        self.grid_height = int(self.map_height / self.map_resolution)
        
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        self.current_path = None
        self.path_cost_value = 0.0
        
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
        
        self.get_logger().info('Path Planner Node initialized')
        self.get_logger().info(f'Grid size: {self.grid_width}x{self.grid_height}')

    def build_grid(self):
        self.smooth_astar.update_grid(self.occupancy_grid)

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
        if self.occupancy_grid[goal[1], goal[0]] >= 50:
            return None
        
        open_set = []
        start_node = Node(start[0], start[1], 0, self.heuristic(start, goal))
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
                tentative_g = g_scores[(current.x, current.y)] + move_cost
                
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    h = self.heuristic(neighbor, goal)
                    neighbor_node = Node(
                        neighbor[0], neighbor[1], 
                        tentative_g, h, current
                    )
                    heapq.heappush(open_set, neighbor_node)
        
        return None

    def smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
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

    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        if not (0 <= goal_grid[0] < self.grid_width and 0 <= goal_grid[1] < self.grid_height):
            self.get_logger().error('Goal position out of bounds')
            return None
        
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
        
        path = self.astar(start_grid, goal_grid)
        
        if path is None:
            self.get_logger().warn('A* failed, trying Dijkstra fallback')
            path = self.dijkstra_fallback(start_grid, goal_grid)
        
        if path is None:
            self.get_logger().error('No path found')
            return None
        
        path = self.smooth_path(path)
        
        world_path = [self.grid_to_world(p[0], p[1]) for p in path]
        
        total_cost = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_cost += math.sqrt(dx * dx + dy * dy) * self.map_resolution
        
        self.path_cost_value = total_cost
        
        return world_path

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
        self.path_cost_value = 0.0

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
        
        cost_msg = std_msgs.Float32()
        cost_msg.data = self.path_cost_value
        self.path_cost_pub.publish(cost_msg)

    def laser_scan_callback(self, msg: sensor_msgs.LaserScan):
        self.update_map_from_laser(msg)

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

    def map_update_callback(self, msg: nav_msgs.OccupancyGrid):
        if msg.info.width != self.grid_width or msg.info.height != self.grid_height:
            self.get_logger().warn('Map update dimensions do not match, ignoring')
            return
        
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.build_grid()
        self.get_logger().info('Map updated from external source')

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
        self.clear_path()
        self.build_grid()
        response.success = True
        response.message = 'Map reset to empty'
        return response


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
