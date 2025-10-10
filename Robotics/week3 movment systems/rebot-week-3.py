import pygame
import random
# Window dimensions
WIDTH, HEIGHT = 600, 400
FPS = 30
# Robot settings
ROBOT_SIZE = 20
ROBOT_SPEED = 5
# Colours
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.direction = random.choice(["UP", "DOWN", "LEFT", "RIGHT"])
   
    def move(self):
        """Move robot in current direction"""
        if self.direction == "UP":
            self.y -= ROBOT_SPEED
        elif self.direction == "DOWN":
            self.y += ROBOT_SPEED
        elif self.direction == "LEFT":
            self.x -= ROBOT_SPEED
        elif self.direction == "RIGHT":
            self.x += ROBOT_SPEED
   
    def detect_collision(self, obstacles):
        """Check collision with obstacles and walls"""
        robot_rect = pygame.Rect(self.x, self.y, ROBOT_SIZE, ROBOT_SIZE)
        
        #wall collisions
        if self.x <= 0 or self.x + ROBOT_SIZE >= WIDTH:
            return True
        if self.y <= 0 or self.y + ROBOT_SIZE >= HEIGHT:
            return True
        
        #obstacle collisions
        for obs in obstacles:
            if robot_rect.colliderect(obs):
                return True
        return False
   
    def react(self):
        """Change direction randomly when hitting obstacle"""
        self.direction = random.choice(["UP", "DOWN", "LEFT", "RIGHT"])
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Reactive Robot Simulation")
    clock = pygame.time.Clock()
   
    #make robots and obstacles
    robot1 = Robot(WIDTH // 2, HEIGHT // 2)
    robot2 = Robot(100, 100)
    obstacles = [
        pygame.Rect(255, 100, 50, 50),
        pygame.Rect(400, 200, 50, 50),
    ]
   
    running = True
    while running:
        screen.fill(WHITE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        #update robots
        robot1.move()
        if robot1.detect_collision(obstacles):
            robot1.react()
        robot2.move()
        if robot2.detect_collision(obstacles):
            robot2.react()
        
        #draw robots
        pygame.draw.rect(screen, BLUE, (robot1.x, robot1.y, ROBOT_SIZE, ROBOT_SIZE))
        pygame.draw.rect(screen, (0, 255, 0), (robot2.x, robot2.y, ROBOT_SIZE, ROBOT_SIZE))
        
        #draw obstacles
        for obs in obstacles:
            pygame.draw.rect(screen, RED, obs)
        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()
if __name__ == "__main__":
    main()