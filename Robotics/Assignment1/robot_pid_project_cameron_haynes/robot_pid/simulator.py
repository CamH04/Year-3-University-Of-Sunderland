import pygame
import math
from robot_pid.planner import AStarPlanner
from robot_pid.robot import Robot
from robot_pid.pid import PIDController

WINDOW_WIDTH = 900
WINDOW_HEIGHT = 600
GRID_SIZE = 20
CELL_SIZE = 25
FPS = 30
DT = 1.00  #TIME STEP (HIGHER = FASTER)

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY = (128, 128, 128)
BLUE = (0, 100, 255)
YELLOW = (255, 255, 100)

PID_CONFIGS = {
    'under_damped': {
        'name': 'Under-damped (High Kp)',
        'heading': {'kp': 3.5, 'ki': 0.01, 'kd': 0.3},
        'speed': {'kp': 1.2, 'ki': 0.05, 'kd': 0.1},
        'color': (255, 100, 100)
    },
    'over_damped': {
        'name': 'Over-damped (High Kd)',
        'heading': {'kp': 0.8, 'ki': 0.001, 'kd': 2.5},
        'speed': {'kp': 0.3, 'ki': 0.001, 'kd': 1.0},
        'color': (100, 150, 255)
    },
    'well_tuned': {
        'name': 'Well-tuned (Balanced)',
        'heading': {'kp': 1.8, 'ki': 0.005, 'kd': 0.8},
        'speed': {'kp': 0.6, 'ki': 0.02, 'kd': 0.3},
        'color': (100, 255, 100)
    }
}

class Simulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Robot PID Controller Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 20)
        self.title_font = pygame.font.Font(None, 28)

        # INIT A*
        self.planner = AStarPlanner(GRID_SIZE, GRID_SIZE)
        self._setup_obstacles()

        self.start_pos = (1, 1)
        self.goal_pos = (GRID_SIZE - 2, GRID_SIZE - 2)

        #FIND PATH
        self.grid_path = self.planner.find_path(self.start_pos, self.goal_pos)
        self.world_path = [(c * CELL_SIZE + CELL_SIZE / 2, r * CELL_SIZE + CELL_SIZE / 2)
                           for r, c in self.grid_path]

        #CUR CONFIG
        self.current_config = 'well_tuned'
        self.robot = None
        self.robot_trail = []

        #SIM STATE
        self.running = False
        self.paused = False
        self.sim_time = 0.0
        self.completed = False
        self.results = {}

        self._initialize_robot()

    def _setup_obstacles(self):
        obstacles = [
            #VIRT WALL
            (5, 5), (5, 6), (5, 7), (5, 8), (5, 9),
            #HORIZ WALL
            (10, 3), (11, 3), (12, 3), (13, 3),
            #SMALL RANDOM
            (15, 10), (15, 11), (15, 12),
            (8, 15), (9, 15), (10, 15),
            #SCATTARED
            (3, 12), (7, 8), (14, 5), (12, 16)
        ]
        for row, col in obstacles:
            if 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE:
                self.planner.set_obstacle(row, col)

    def _initialize_robot(self):
        config = PID_CONFIGS[self.current_config]

        #INIT BOT
        start_x = self.start_pos[1] * CELL_SIZE + CELL_SIZE / 2
        start_y = self.start_pos[0] * CELL_SIZE + CELL_SIZE / 2
        self.robot = Robot(start_x, start_y)

        #INIT PID
        heading_pid = PIDController(**config['heading'])
        speed_pid = PIDController(**config['speed'])
        self.robot.set_pid_controllers(heading_pid, speed_pid)

        # SET PAHT
        self.robot.set_path(self.world_path)
        self.robot.start_time = 0.0

        # STATE RESET
        self.robot_trail = []
        self.sim_time = 0.0
        self.completed = False
        self.results = {}

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if not self.running:
                        self.running = True
                    else:
                        self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self._initialize_robot()
                    self.running = False
                    self.paused = False
                elif event.key == pygame.K_1:
                    self.current_config = 'under_damped'
                    self._initialize_robot()
                    self.running = False
                elif event.key == pygame.K_2:
                    self.current_config = 'over_damped'
                    self._initialize_robot()
                    self.running = False
                elif event.key == pygame.K_3:
                    self.current_config = 'well_tuned'
                    self._initialize_robot()
                    self.running = False

        return True

    def update(self):
        if not self.running or self.paused or self.completed:
            return

        self.sim_time += DT

        goal_reached = self.robot.update(DT)

        #trail chceking =============
        self.robot_trail.append((self.robot.x, self.robot.y))
        if len(self.robot_trail) > 1000:
            self.robot_trail.pop(0)

        if goal_reached:
            self.completed = True
            self.results = self.robot.get_metrics(self.sim_time)
            print(f"\n{'='*60}")
            print(f"Configuration: {PID_CONFIGS[self.current_config]['name']}")
            print(f"{'='*60}")
            for key, value in self.results.items():
                print(f"{key}: {value:.3f}")
            print(f"{'='*60}\n")

    def draw(self):

        self.screen.fill(WHITE)

        #GRID =============
        for i in range(GRID_SIZE + 1):
            pygame.draw.line(self.screen, (220, 220, 220),
                           (i * CELL_SIZE, 0), (i * CELL_SIZE, GRID_SIZE * CELL_SIZE))
            pygame.draw.line(self.screen, (220, 220, 220),
                           (0, i * CELL_SIZE), (GRID_SIZE * CELL_SIZE, i * CELL_SIZE))

        #OBS ==========================
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                if self.planner.grid[row][col].is_obstacle:
                    rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                    pygame.draw.rect(self.screen, GRAY, rect)

        #PATH ================
        if len(self.world_path) > 1:
            pygame.draw.lines(self.screen, BLUE, False, self.world_path, 2)
        start_rect = pygame.Rect(self.start_pos[1] * CELL_SIZE, self.start_pos[0] * CELL_SIZE,
                                CELL_SIZE, CELL_SIZE)
        goal_rect = pygame.Rect(self.goal_pos[1] * CELL_SIZE, self.goal_pos[0] * CELL_SIZE,
                               CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(self.screen, GREEN, start_rect)
        pygame.draw.rect(self.screen, RED, goal_rect)
        if len(self.robot_trail) > 1:
            color = PID_CONFIGS[self.current_config]['color']
            for i in range(len(self.robot_trail) - 1):
                pygame.draw.line(self.screen, color, self.robot_trail[i], self.robot_trail[i + 1], 2)
        color = PID_CONFIGS[self.current_config]['color']
        robot_pos = (int(self.robot.x), int(self.robot.y))
        pygame.draw.circle(self.screen, color, robot_pos, 8)
        heading_end = (
            int(self.robot.x + 15 * math.cos(self.robot.theta)),
            int(self.robot.y + 15 * math.sin(self.robot.theta)) )
        pygame.draw.line(self.screen, BLACK, robot_pos, heading_end, 3)
        self._draw_ui()
        pygame.display.flip()

    def _draw_ui(self):
        ui_x = GRID_SIZE * CELL_SIZE + 20
        y = 20
        title = self.title_font.render("PID Robot Simulator", True, BLACK)
        self.screen.blit(title, (ui_x, y))
        y += 40

        config = PID_CONFIGS[self.current_config]
        config_text = self.font.render(f"Config: {config['name']}", True, BLACK)
        self.screen.blit(config_text, (ui_x, y))
        y += 30

        y = self._draw_text(ui_x, y, "Heading PID:", BLACK, bold=True)
        y = self._draw_text(ui_x + 10, y, f"Kp: {config['heading']['kp']:.2f}", BLACK)
        y = self._draw_text(ui_x + 10, y, f"Ki: {config['heading']['ki']:.3f}", BLACK)
        y = self._draw_text(ui_x + 10, y, f"Kd: {config['heading']['kd']:.2f}", BLACK)
        y += 10
        y = self._draw_text(ui_x, y, "Speed PID:", BLACK, bold=True)
        y = self._draw_text(ui_x + 10, y, f"Kp: {config['speed']['kp']:.2f}", BLACK)
        y = self._draw_text(ui_x + 10, y, f"Ki: {config['speed']['ki']:.3f}", BLACK)
        y = self._draw_text(ui_x + 10, y, f"Kd: {config['speed']['kd']:.2f}", BLACK)
        y += 20

        status = "COMPLETED" if self.completed else "PAUSED" if self.paused else "RUNNING" if self.running else "READY"
        status_color = RED if self.completed else BLUE if self.paused else GREEN if self.running else BLACK
        y = self._draw_text(ui_x, y, f"Status: {status}", status_color, bold=True)
        y = self._draw_text(ui_x, y, f"Time: {self.sim_time:.2f}s", BLACK)
        y += 20
        if self.results:
            y = self._draw_text(ui_x, y, "Results:", BLACK, bold=True)
            y = self._draw_text(ui_x + 10, y, f"Time: {self.results['time_to_goal']:.2f}s", BLACK)
            y = self._draw_text(ui_x + 10, y, f"Distance: {self.results['path_length']:.1f}px", BLACK)
            y = self._draw_text(ui_x + 10, y, f"Avg Error: {self.results['avg_cross_track_error']:.2f}", BLACK)
            y = self._draw_text(ui_x + 10, y, f"Max Error: {self.results['max_cross_track_error']:.2f}", BLACK)

        y = WINDOW_HEIGHT - 150
        y = self._draw_text(ui_x, y, "Controls:", BLACK, bold=True)
        y = self._draw_text(ui_x + 10, y, "SPACE - Start/Pause", BLACK)
        y = self._draw_text(ui_x + 10, y, "R - Reset", BLACK)
        y = self._draw_text(ui_x + 10, y, "1 - Under-damped", BLACK)
        y = self._draw_text(ui_x + 10, y, "2 - Over-damped", BLACK)
        y = self._draw_text(ui_x + 10, y, "3 - Well-tuned", BLACK)

    def _draw_text(self, x, y, text, color, bold=False):
        font = self.title_font if bold else self.font
        surface = font.render(text, True, color)
        self.screen.blit(surface, (x, y))
        return y + 25

    def run(self):
        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)
        pygame.quit()
