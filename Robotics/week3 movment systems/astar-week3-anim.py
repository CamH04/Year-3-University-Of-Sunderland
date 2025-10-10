import pygame
import heapq
import random

#grid settings
WIDTH, HEIGHT = 800, 600
ROWS, COLS = 100, 100
CELL_SIZE = min(WIDTH // (COLS + 10), HEIGHT // ROWS)
FPS = 10

#colours
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0) 
RED = (255, 0, 0)
BLUE = (0, 0, 255) 
PURPLE = (160, 32, 240)  
ORANGE = (255, 165, 0)  
GRAY = (128, 128, 128)  
YELLOW = (255, 255, 0) 

class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.g = float("inf")
        self.h = 0
        self.f = float("inf")
        self.parent = None
        self.is_obstacle = False
    
    def __lt__(self, other):
        return self.f < other.f
    
    def reset(self):
        self.g = float("inf")
        self.h = 0
        self.f = float("inf")
        self.parent = None

class DynamicObstacle:
    def __init__(self, row, col, dr, dc):
        self.row = row
        self.col = col
        self.dr = dr  # row dir
        self.dc = dc  #coloum dir
    
    def move(self):
        new_row = self.row + self.dr
        new_col = self.col + self.dc
        #dont crash into wall ()
        if new_row < 0 or new_row >= ROWS:
            self.dr *= -1
        if new_col < 0 or new_col >= COLS:
            self.dc *= -1
        
        self.row = max(0, min(ROWS - 1, new_row))
        self.col = max(0, min(COLS - 1, new_col))

def heuristic(a, b):
    return abs(a.row - b.row) + abs(a.col - b.col)

def a_star(grid, start, goal):
    # reset nodes
    for row in grid:
        for node in row:
            node.reset()
    open_list = []
    heapq.heappush(open_list, (0, start))
    start.g = 0
    start.f = heuristic(start, goal)
    closed_set = set()
    while open_list:
        _, current = heapq.heappop(open_list)
        if current in closed_set:
            continue
        closed_set.add(current)
        if current == goal:
            return reconstruct_path(goal)
        for neighbor in get_neighbors(grid, current):
            if neighbor.is_obstacle or neighbor in closed_set:
                continue
            temp_g = current.g + 1
            if temp_g < neighbor.g:
                neighbor.g = temp_g
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current
                heapq.heappush(open_list, (neighbor.f, neighbor))
    return []

def get_neighbors(grid, node):
    neighbors = []
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for d in directions:
        r, c = node.row + d[0], node.col + d[1]
        if 0 <= r < ROWS and 0 <= c < COLS:
            neighbors.append(grid[r][c])
    return neighbors

def reconstruct_path(goal):
    path = []
    current = goal
    while current.parent:
        path.append((current.row, current.col))
        current = current.parent
    return path[::-1]

def reactive_move(grid, current_pos, goal, obstacles):
    """Reactive control: move toward goal while avoiding obstacles"""
    r, c = current_pos
    goal_r, goal_c = goal.row, goal.col
    moves = []
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    
    for dr, dc in directions:
        new_r, new_c = r + dr, c + dc
        if 0 <= new_r < ROWS and 0 <= new_c < COLS:
            #blocked by static or dynamic obstacle?
            if grid[new_r][new_c].is_obstacle:
                continue
            if any(obs.row == new_r and obs.col == new_c for obs in obstacles):
                continue
            #distance to goal
            dist = abs(new_r - goal_r) + abs(new_c - goal_c)
            moves.append((dist, new_r, new_c))
    if not moves:
        return current_pos  #stuck lmao
    #choose move that gets closest to finish
    moves.sort()
    _, new_r, new_c = moves[0]
    return (new_r, new_c)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Deliberative (Purple) vs Reactive (Orange) Control")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)
    
    #===grid===
    grid = [[Node(r, c) for c in range(COLS)] for r in range(ROWS)]
    start = grid[1][1]
    goal = grid[ROWS - 2][COLS - 2]
    #make static obstacles
    for _ in range(15):
        r, c = random.randint(0, ROWS - 1), random.randint(0, COLS - 1)
        if (r, c) != (start.row, start.col) and (r, c) != (goal.row, goal.col):
            grid[r][c].is_obstacle = True
    #make dynamic obstacles
    dynamic_obstacles = []
    for _ in range(5):
        r = random.randint(3, ROWS - 4)
        c = random.randint(3, COLS - 4)
        dr = random.choice([-1, 1])
        dc = random.choice([-1, 1])
        dynamic_obstacles.append(DynamicObstacle(r, c, dr, dc))
    #init robots
    deliberative_pos = (start.row, start.col)
    reactive_pos = (start.row, start.col)
    deliberative_path = a_star(grid, start, goal)
    deliberative_index = 0
    #===stats===
    deliberative_replans = 0
    deliberative_stuck = False
    reactive_stuck = False
    deliberative_reached = False
    reactive_reached = False
    frame_count = 0
    running = True

    while running:
        screen.fill(WHITE)        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        for obs in dynamic_obstacles:
            obs.move()
        frame_count += 1
        for r in range(ROWS):
            for c in range(COLS):
                x = c * CELL_SIZE
                y = r * CELL_SIZE
                rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(screen, BLACK, rect, 1)
                if grid[r][c].is_obstacle:
                    pygame.draw.rect(screen, GRAY, rect)
        for obs in dynamic_obstacles:
            rect = pygame.Rect(obs.col * CELL_SIZE, obs.row * CELL_SIZE, 
                             CELL_SIZE, CELL_SIZE)
            pygame.draw.circle(screen, YELLOW, rect.center, CELL_SIZE // 3)
        pygame.draw.rect(screen, GREEN, (start.col * CELL_SIZE, start.row * CELL_SIZE,
                        CELL_SIZE, CELL_SIZE))
        pygame.draw.rect(screen, RED, (goal.col * CELL_SIZE, goal.row * CELL_SIZE,
                        CELL_SIZE, CELL_SIZE))
        if not deliberative_reached and not deliberative_stuck:
            path_blocked = False
            if deliberative_index < len(deliberative_path):
                next_pos = deliberative_path[deliberative_index]
                for obs in dynamic_obstacles:
                    if obs.row == next_pos[0] and obs.col == next_pos[1]:
                        path_blocked = True
                        break
            
            if path_blocked or deliberative_index >= len(deliberative_path):
                deliberative_replans += 1
                current_node = grid[deliberative_pos[0]][deliberative_pos[1]]
                deliberative_path = a_star(grid, current_node, goal)
                deliberative_index = 0
                if not deliberative_path:
                    deliberative_stuck = True
            if deliberative_index < len(deliberative_path):
                deliberative_pos = deliberative_path[deliberative_index]
                deliberative_index += 1
                if deliberative_pos == (goal.row, goal.col):
                    deliberative_reached = True
        
        if not deliberative_stuck:
            pygame.draw.rect(screen, PURPLE, 
                           (deliberative_pos[1] * CELL_SIZE, deliberative_pos[0] * CELL_SIZE,
                            CELL_SIZE, CELL_SIZE))
            
        if not reactive_reached and not reactive_stuck:
            new_pos = reactive_move(grid, reactive_pos, goal, dynamic_obstacles)
            if new_pos == reactive_pos and reactive_pos != (goal.row, goal.col):
                reactive_stuck = True
            else:
                reactive_pos = new_pos
                if reactive_pos == (goal.row, goal.col):
                    reactive_reached = True

        if not reactive_stuck:
            pygame.draw.circle(screen, ORANGE,
                             (reactive_pos[1] * CELL_SIZE + CELL_SIZE // 2,
                              reactive_pos[0] * CELL_SIZE + CELL_SIZE // 2),
                             CELL_SIZE // 3)
        
        info_x = COLS * CELL_SIZE + 10
        stats = [
            "DELIBERATIVE (Purple Square):",
            f"  Replans: {deliberative_replans}",
            f"  Status: {'REACHED!' if deliberative_reached else 'STUCK' if deliberative_stuck else 'Moving'}",
            "",
            "REACTIVE (Orange Circle):",
            f"  Status: {'REACHED!' if reactive_reached else 'STUCK' if reactive_stuck else 'Moving'}",
            "",
            "Yellow circles = moving obstacles",
            "Gray squares = static obstacles"
        ]
        y_offset = 20
        for line in stats:
            text = font.render(line, True, BLACK)
            screen.blit(text, (info_x, y_offset))
            y_offset += 30
        
        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()

if __name__ == "__main__":
    main()