import heapq
import math
from typing import List, Tuple, Optional


class SmoothAStar:
    def __init__(self, grid, width: int, height: int, heuristic_weight: float = 1.0):
        self.grid = grid
        self.width = width
        self.height = height
        self.heuristic_weight = heuristic_weight

    def update_grid(self, grid):
        self.grid = grid

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
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny, nx] < 50:
                    if dx != 0 and dy != 0:
                        if self.grid[node[1], node[0] + dx] >= 50 or \
                           self.grid[node[1] + dy, node[0]] >= 50:
                            continue
                    neighbors.append((nx, ny))
        return neighbors

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
            if self.grid[y, x] >= 50:
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

    def add_smooth_turns(self, path: List[Tuple[int, int]], turn_threshold: float = 0.5) -> List[Tuple[int, int]]:
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev = smoothed[-1]
            curr = path[i]
            next_p = path[i + 1]
            
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_p[1] - curr[1], next_p[0] - curr[0])
            
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff > turn_threshold:
                smoothed.append(curr)
        
        smoothed.append(path[-1])
        
        return smoothed

    def optimize_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path
        
        optimized = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            farthest = current_idx + 1
            
            for check_idx in range(len(path) - 1, current_idx, -1):
                if self.check_line_of_sight(path[current_idx], path[check_idx]):
                    farthest = check_idx
                    break
            
            optimized.append(path[farthest])
            current_idx = farthest
        
        return optimized

    def calculate_curvature(self, path: List[Tuple[int, int]]) -> float:
        if len(path) < 3:
            return 0.0
        
        total_curvature = 0.0
        count = 0
        
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_p = path[i + 1]
            
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_p[1] - curr[1], next_p[0] - curr[0])
            
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            total_curvature += angle_diff
            count += 1
        
        return total_curvature / count if count > 0 else 0.0

    def replan_if_needed(self, current_pos: Tuple[float, float], path: List[Tuple[int, int]], 
                         obstacles: List[Tuple[int, int]], threshold: float = 2.0) -> bool:
        if not path or len(path) < 2:
            return True
        
        for i, node in enumerate(path):
            if self.grid[node[1], node[0]] >= 50:
                return True
            
            if i > 0:
                dist = math.sqrt(
                    (node[0] - current_pos[0]) ** 2 + 
                    (node[1] - current_pos[1]) ** 2
                )
                if dist < threshold:
                    return False
        
        curvature = self.calculate_curvature(path)
        if curvature > math.pi / 2:
            return True
        
        return False

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        if self.grid[goal[1], goal[0]] >= 50:
            return None
        
        open_set = []
        start_node = _AStarNode(start[0], start[1], 0, self.heuristic(start, goal))
        heapq.heappush(open_set, start_node)
        
        closed_set = set()
        g_scores = {start: 0}
        
        while open_set:
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
                    neighbor_node = _AStarNode(
                        neighbor[0], neighbor[1], 
                        tentative_g, h, current
                    )
                    heapq.heappush(open_set, neighbor_node)
        
        return None


class _AStarNode:
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: '_AStarNode' = None):
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
