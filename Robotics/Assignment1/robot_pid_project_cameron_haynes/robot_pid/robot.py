import math
from typing import Tuple, List, Optional
from robot_pid.pid import PIDController


class Robot:
    """
    uses two PID controllers:
    - Heading controller: Controls angular velocity to face desired direction
    - Speed controller: Controls linear velocity to maintain desired speed

    Attributes:
        x (float): X position in world coordinates
        y (float): Y position in world coordinates
        theta (float): Heading angle in radians
        v (float): Linear velocity
        omega (float): Angular velocity
        max_speed (float): Maximum linear velocity
        max_angular_velocity (float): Maximum angular velocity
    """

    def __init__(self, x: float, y: float, theta: float = 0.0,
                 max_speed: float = 3.0, max_angular_velocity: float = 2.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.max_speed = max_speed
        self.max_angular_velocity = max_angular_velocity

        self.heading_pid: Optional[PIDController] = None
        self.speed_pid: Optional[PIDController] = None

        self.path: List[Tuple[float, float]] = []
        self.path_index = 0

        self.total_distance = 0.0
        self.cross_track_errors: List[float] = []
        self.start_time = 0.0
        self.completion_time: Optional[float] = None

    def set_pid_controllers(self, heading_pid: PIDController, speed_pid: PIDController):
        self.heading_pid = heading_pid
        self.speed_pid = speed_pid

    def set_path(self, path: List[Tuple[float, float]]):
        self.path = path
        self.path_index = 0
        self.total_distance = 0.0
        self.cross_track_errors = []
        if self.heading_pid:
            self.heading_pid.reset()
        if self.speed_pid:
            self.speed_pid.reset()

    def update(self, dt: float) -> bool:
        if not self.path or self.path_index >= len(self.path):
            return True

        if not self.heading_pid or not self.speed_pid:
            raise ValueError("PID controllers not set. Call set_pid_controllers() first.")

        target_x, target_y = self.path[self.path_index]

        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.2:
            self.path_index += 1
            if self.path_index >= len(self.path):
                return True
            target_x, target_y = self.path[self.path_index]
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.sqrt(dx * dx + dy * dy)

        desired_theta = math.atan2(dy, dx)

        heading_error = desired_theta - self.theta
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        desired_speed = min(self.max_speed, distance * 0.5)
        speed_error = desired_speed - self.v

        angular_control = self.heading_pid.compute(heading_error, dt)
        linear_control = self.speed_pid.compute(speed_error, dt)

        self.omega = self._clamp(angular_control, -self.max_angular_velocity, self.max_angular_velocity)
        acceleration = linear_control
        self.v = self._clamp(self.v + acceleration * dt, 0, self.max_speed)

        prev_x, prev_y = self.x, self.y
        self.theta += self.omega * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        step_distance = math.sqrt((self.x - prev_x)**2 + (self.y - prev_y)**2)
        self.total_distance += step_distance

        cross_track = abs(dx * math.sin(self.theta) - dy * math.cos(self.theta))
        self.cross_track_errors.append(cross_track)

        return False

    def get_metrics(self, current_time: float) -> dict:
        if not self.cross_track_errors:
            return {}
        return {
            'time_to_goal': current_time - self.start_time,
            'path_length': self.total_distance,
            'avg_cross_track_error': sum(self.cross_track_errors) / len(self.cross_track_errors),
            'max_cross_track_error': max(self.cross_track_errors),
            'num_samples': len(self.cross_track_errors)
        }

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

