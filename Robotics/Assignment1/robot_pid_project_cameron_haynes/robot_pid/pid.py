# ============================================================================
# robot_pid/pid.py - PID Controller Implementation
# ============================================================================

class PIDController:

        #kp: New proportional gain
        #ki: New integral gain
        #kd: New derivative gain
        #integral: Accumulated integral term
        #prev_error: Previous error for derivative calculation
        #integral_limit : Maximum absolute value for integral term (anti-windup)


    def __init__(self, kp, ki, kd, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def compute(self, error, dt):
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        self.prev_error = error
        return p_term + i_term + d_term

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
