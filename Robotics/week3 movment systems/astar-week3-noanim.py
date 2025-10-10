# Deliberative Robot Control with A* Path Planning
import pygame
import heapq

# Grid settings
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 20, 20
CELL_SIZE = WIDTH // COLS
FPS = 10

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)  # Start
RED = (255, 0, 0)    # Goal
BLUE = (0, 0, 255)   # Path

class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.g = float("inf")  # Cost from start
        self.h = 0              # Heuristic cost
        self.f = float("inf")  # Total cost
        self.parent = None      # Used to reconstruct path
    
    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    """Manhattan distance heuristic"""
    return abs(a.row - b.row) + abs(a.col - b.col)

def a_star(grid, start, goal):
    """A* algorithm implementation"""
    open_list = []
    heapq.heappush(open_list, (0, start))
    start.g = 0
    start.f = heuristic(start, goal)
    
    while open_list:
        _, current = heapq.heappop(open_list)
        
        if current == goal:
            return reconstruct_path(goal)
        
        for neighbor in get_neighbors(grid, current):
            temp_g = current.g + 1
            if temp_g < neighbor.g:
                neighbor.g = temp_g
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current
                heapq.heappush(open_list, (neighbor.f, neighbor))
    
    return []

def get_neighbors(grid, node):
    """Get valid neighbors in the 4-connected grid"""
    neighbors = []
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for d in directions:
        r, c = node.row + d[0], node.col + d[1]
        if 0 <= r < ROWS and 0 <= c < COLS:
            neighbors.append(grid[r][c])
    return neighbors

def reconstruct_path(goal):
    """Reconstruct path from goal to start"""
    path = []
    current = goal
    while current.parent:
        path.append((current.row, current.col))
        current = current.parent
    return path[::-1]  # Reverse → start to goal

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Deliberative Robot with A* Path Planning")
    clock = pygame.time.Clock()
    
    # Create grid of nodes
    grid = [[Node(r, c) for c in range(COLS)] for r in range(ROWS)]
    start = grid[0][0]
    goal = grid[ROWS - 1][COLS - 1]
    print(f"Start: ({start.row}, {start.col}), Goal: ({goal.row}, {goal.col})")
    
    # Run A* path planning
    path = a_star(grid, start, goal)
    
    running = True
    while running:
        screen.fill(WHITE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Draw grid
        for r in range(ROWS):
            for c in range(COLS):
                rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(screen, BLACK, rect, 1)
        
        # Draw path FIRST
        for (r, c) in path:
            pygame.draw.rect(screen, BLUE, (c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        
        # Draw start and goal LAST so they are always visible
        pygame.draw.rect(screen, GREEN, (start.col * CELL_SIZE, start.row * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        pygame.draw.rect(screen, RED, (goal.col * CELL_SIZE, goal.row * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()

if __name__ == "__main__":
    main()