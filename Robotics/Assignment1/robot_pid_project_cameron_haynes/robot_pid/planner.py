import heapq
from typing import List, Tuple, Optional

class Node:

    def __init__(self, row: int, col: int):
        self.row = row
        self.col = col
        self.g = float('inf')
        self.h = 0.0
        self.f = float('inf')
        self.parent: Optional[Node] = None
        self.is_obstacle = False

    def __lt__(self, other):
        return self.f < other.f

    def reset(self):
        self.g = float('inf')
        self.h = 0.0
        self.f = float('inf')
        self.parent = None


class AStarPlanner:
    # A* IMP

    def __init__(self, grid_rows: int, grid_cols: int):
        self.rows = grid_rows
        self.cols = grid_cols
        self.grid = [[Node(r, c) for c in range(grid_cols)] for r in range(grid_rows)]

    def set_obstacle(self, row: int, col: int):
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.grid[row][col].is_obstacle = True

    def clear_obstacle(self, row: int, col: int):
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.grid[row][col].is_obstacle = False

    @staticmethod
    def heuristic(node_a: Node, node_b: Node) -> float:
        return abs(node_a.row - node_b.row) + abs(node_a.col - node_b.col)

    def get_neighbors(self, node: Node) -> List[Node]:
        neighbors = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dr, dc in directions:
            r, c = node.row + dr, node.col + dc
            if 0 <= r < self.rows and 0 <= c < self.cols:
                neighbors.append(self.grid[r][c])
        return neighbors

    def find_path(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        for row in self.grid:
            for node in row:
                node.reset()
        start = self.grid[start_pos[0]][start_pos[1]]
        goal = self.grid[goal_pos[0]][goal_pos[1]]
        if start.is_obstacle or goal.is_obstacle:
            return []
        open_list = []
        closed_set = set()
        start.g = 0
        start.f = self.heuristic(start, goal)
        heapq.heappush(open_list, start)

        while open_list:
            current = heapq.heappop(open_list)
            if current in closed_set:
                continue
            closed_set.add(current)
            if current == goal:
                return self._reconstruct_path(goal)
            for neighbor in self.get_neighbors(current):
                if neighbor.is_obstacle or neighbor in closed_set:
                    continue
                tentative_g = current.g + 1
                if tentative_g < neighbor.g:
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(neighbor, goal)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current
                    heapq.heappush(open_list, neighbor)

        return []

    def _reconstruct_path(self, goal: Node) -> List[Tuple[int, int]]:
        path = []
        current = goal

        while current.parent is not None:
            path.append((current.row, current.col))
            current = current.parent

        path.reverse()
        return path
