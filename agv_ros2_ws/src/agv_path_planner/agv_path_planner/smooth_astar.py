# 平滑A*路径规划算法实现
# 提供A*搜索、路径平滑优化、转弯平滑、曲率计算和重规划判断等功能
import heapq
import math
from typing import List, Tuple, Optional


# 平滑A*规划器类，封装路径搜索和优化算法
class SmoothAStar:
    # 初始化规划器，传入栅格地图和参数
    def __init__(self, grid, width: int, height: int, heuristic_weight: float = 1.0):
        self.grid = grid
        self.width = width
        self.height = height
        self.heuristic_weight = heuristic_weight

    # 更新栅格地图数据
    def update_grid(self, grid):
        self.grid = grid

    # 启发式函数，计算两节点间的加权欧几里得距离
    def heuristic(self, node1: Tuple[int, int], node2: Tuple[int, int]) -> float:
        dx = abs(node1[0] - node2[0])
        dy = abs(node1[1] - node2[1])
        return math.sqrt(dx * dx + dy * dy) * self.heuristic_weight

    # 获取节点的8邻域邻居，过滤障碍物和对角线穿越
    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        # 8方向移动
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # 检查邻居是否为空闲格子
                if self.grid[ny, nx] < 50:
                    # 对角线移动时检查相邻格子，防止穿越障碍物角落
                    if dx != 0 and dy != 0:
                        if self.grid[node[1], node[0] + dx] >= 50 or \
                           self.grid[node[1] + dy, node[0]] >= 50:
                            continue
                    neighbors.append((nx, ny))
        return neighbors

    # 视线检测，使用Bresenham算法判断两点间是否有障碍物
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
            if self.grid[y, x] >= 50:
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

    # 添加平滑转弯，移除角度变化较小的中间路径点
    def add_smooth_turns(self, path: List[Tuple[int, int]], turn_threshold: float = 0.5) -> List[Tuple[int, int]]:
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = smoothed[-1]
            curr = path[i]
            next_p = path[i + 1]
            
            # 计算相邻两段路径的方向角
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_p[1] - curr[1], next_p[0] - curr[0])
            
            # 计算角度差
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # 只保留角度变化超过阈值的转弯点
            if angle_diff > turn_threshold:
                smoothed.append(curr)
        
        smoothed.append(path[-1])
        
        return smoothed

    # 路径优化，通过视线检测移除不必要的中间路径点
    def optimize_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path
        
        optimized = [path[0]]
        current_idx = 0
        
        # 贪心策略：从当前点找到最远的可直接到达的路径点
        while current_idx < len(path) - 1:
            farthest = current_idx + 1
            
            for check_idx in range(len(path) - 1, current_idx, -1):
                if self.check_line_of_sight(path[current_idx], path[check_idx]):
                    farthest = check_idx
                    break
            
            optimized.append(path[farthest])
            current_idx = farthest
        
        return optimized

    # 计算路径的平均曲率，用于评估路径平滑度
    def calculate_curvature(self, path: List[Tuple[int, int]]) -> float:
        if len(path) < 3:
            return 0.0
        
        total_curvature = 0.0
        count = 0
        
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_p = path[i + 1]
            
            # 计算相邻两段路径的方向角变化
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_p[1] - curr[1], next_p[0] - curr[0])
            
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            total_curvature += angle_diff
            count += 1
        
        return total_curvature / count if count > 0 else 0.0

    # 判断是否需要重新规划路径
    def replan_if_needed(self, current_pos: Tuple[float, float], path: List[Tuple[int, int]], 
                         obstacles: List[Tuple[int, int]], threshold: float = 2.0) -> bool:
        if not path or len(path) < 2:
            return True
        
        for i, node in enumerate(path):
            # 路径上有障碍物则需要重规划
            if self.grid[node[1], node[0]] >= 50:
                return True
            
            # 检查当前位置是否接近路径点
            if i > 0:
                dist = math.sqrt(
                    (node[0] - current_pos[0]) ** 2 + 
                    (node[1] - current_pos[1]) ** 2
                )
                if dist < threshold:
                    return False
        
        # 路径曲率过大则需要重规划
        curvature = self.calculate_curvature(path)
        if curvature > math.pi / 2:
            return True
        
        return False

    # A*路径搜索算法，在栅格地图上寻找最短路径
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        # 目标点在障碍物中则无法到达
        if self.grid[goal[1], goal[0]] >= 50:
            return None
        
        open_set = []
        start_node = _AStarNode(start[0], start[1], 0, self.heuristic(start, goal))
        heapq.heappush(open_set, start_node)
        
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
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
                
                # 计算移动代价
                move_cost = math.sqrt(
                    (neighbor[0] - current.x) ** 2 + 
                    (neighbor[1] - current.y) ** 2
                )
                tentative_g = g_scores[(current.x, current.y)] + move_cost
                
                # 找到更优路径或首次访问该节点
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    h = self.heuristic(neighbor, goal)
                    neighbor_node = _AStarNode(
                        neighbor[0], neighbor[1], 
                        tentative_g, h, current
                    )
                    heapq.heappush(open_set, neighbor_node)
        
        return None


# A*搜索内部节点类，存储位置、代价和父节点信息
class _AStarNode:
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: '_AStarNode' = None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    # 比较运算符，用于优先队列排序
    def __lt__(self, other):
        return self.f < other.f

    # 相等判断，基于坐标
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    # 哈希值，用于集合去重
    def __hash__(self):
        return hash((self.x, self.y))
