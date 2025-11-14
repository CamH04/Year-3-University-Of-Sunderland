# dwa_planner.py

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ---- 1. CONFIGURATION AND ROBOT PARAMETERS ----
# State Vector: [x, y, yaw, velocity, angular_velocity]

DT = 0.1
ROBOT_RADIUS = 0.40     # reduced from 1.0
MAX_SPEED = 1.0
MAX_YAWRATE = 1.0
MAX_ACCEL = 1.0
MAX_D_YAWRATE = 0.5
V_RESOLUTION = 0.02
YAW_RESOLUTION = 0.1
PREDICT_TIME = 3.0

WEIGHT_HEADING = 2.0      # increased from 0.1 → reduces spinning
WEIGHT_DISTANCE = 0.2     # reduced from 1.0 → obstacle avoidance less dominant
WEIGHT_VELOCITY = 0.1
WEIGHT_BRAKE = 1000.0


# ---- 2. CORE DWA FUNCTIONS ----

def dwa_planning(x, goal, ob, config):
    """
    Core DWA function to find the optimal (v, w) command.
    """
    # 1. Determine Dynamic Window (Admissible Velocities)
    dw = calc_dynamic_window(x, config)

    # 2. Evaluate all Trajectories within DW
    best_u, best_trajectory = calc_control_and_trajectory(x, dw, goal, ob, config)

    # Return the optimal velocity command (v, w) and the planned trajectory
    return best_u, best_trajectory

def calc_dynamic_window(x, config):
    """
    Calculates the feasible velocity window based on:
    1. Robot's kinematic limits (Max Speed/YawRate)
    2. Robot's dynamic limits (Max Acceleration over DT)
    """
    # [Vmin, Vmax, YawMin, YawMax]

    # 1. Kinematic Limits (V_s)
    Vs = [0, config.max_speed, -config.max_yawrate, config.max_yawrate]

    # 2. Dynamic Limits (V_d): Current speed +/- acceleration over one time step
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_d_yawrate * config.dt,
          x[4] + config.max_d_yawrate * config.dt]

    # Combine limits: Dynamic Window (Dw) is the intersection of Vs and Vd
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

def calc_control_and_trajectory(x, dw, goal, ob, config):
    """
    Samples velocities in the Dynamic Window and evaluates each resulting trajectory.
    """
    x_init = x[:]
    best_u = [0.0, 0.0]  # [v, w]
    min_cost = float("inf")
    best_trajectory = np.array(x)

    # Sample linear velocity (v)
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        # Sample angular velocity (w)
        for w in np.arange(dw[2], dw[3], config.yaw_resolution):

            # Predict the trajectory resulting from (v, w)
            trajectory = generate_trajectory(x_init, v, w, config)

            # Check for collision risk (DWA's core safety check)
            min_ob_dist = calc_obstacle_cost(trajectory, ob, config)

            if min_ob_dist == 0.0:  # Collision predicted
                continue  # Skip this velocity pair

            # Calculate the cost function for this trajectory

            # Heading cost: Alignment with goal direction
            heading_cost = config.weight_heading * calc_heading_cost(trajectory, goal)

            # Clearance cost: Inverse of distance to the nearest obstacle (safety)
            # Bounded obstacle cost using exponential decay
            dist_cost = config.weight_distance * math.exp(-min_ob_dist)


            # Velocity cost: Encourages higher speed
            vel_cost = config.weight_velocity * (config.max_speed - v)

            # Total Cost
            final_cost = heading_cost + dist_cost + vel_cost

            # Check for minimum cost
            if final_cost < min_cost:
                min_cost = final_cost
                # Enforce minimum forward motion (avoid spinning)
                v_cmd = max(v, config.min_speed)
                best_u = [v_cmd, w]
                best_trajectory = trajectory

    # If no feasible trajectory is found (shouldn't happen if max_speed > 0)
    if min_cost == float("inf"):
        return [0.0, 0.0], np.array(x_init)

    return best_u, best_trajectory

# ---- 3. KINEMATICS AND COST FUNCTIONS ----

def generate_trajectory(x, v, w, config):
    """
    Simulates the robot's motion over the prediction time.
    """
    time = 0.0
    traj = np.array(x)
    state = np.array(x)

    while time <= config.predict_time:
        # Simple Unicycle Kinematics:
        state = state + np.array([
            v * math.cos(state[2]) * config.dt,
            v * math.sin(state[2]) * config.dt,
            w * config.dt,
            0.0,  # Linear velocity is assumed constant for this prediction
            0.0   # Angular velocity is assumed constant for this prediction
        ])
        state[2] = normalize_angle(state[2])  # Keep yaw between -pi and pi

        traj = np.vstack((traj, state))
        time += config.dt

    return traj

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_heading_cost(trajectory, goal):
    """
    Calculates the cost based on the difference between the robot's final heading
    in the trajectory and the angle to the goal. Lower is better.
    """
    # 1. Angle from robot position to goal (target_yaw)
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    target_yaw = math.atan2(dy, dx)

    # 2. Robot's final heading in the trajectory (final_yaw)
    final_yaw = trajectory[-1, 2]

    # 3. Heading error
    heading_error = abs(normalize_angle(target_yaw - final_yaw))

    return heading_error

def calc_obstacle_cost(trajectory, ob, config):
    """
    Calculates the minimum distance from the predicted trajectory to the nearest obstacle.
    """
    min_dist = float("inf")

    # Iterate through all points in the predicted trajectory
    for i in range(trajectory.shape[0]):
        # Calculate distance to all obstacles
        for j in range(len(ob)):
            dx = trajectory[i, 0] - ob[j][0]
            dy = trajectory[i, 1] - ob[j][1]
            dist = math.sqrt(dx**2 + dy**2) - config.robot_radius

            # Check for immediate collision (dist <= 0)
            if dist <= 0:
                return 0.0  # Return 0.0 distance cost for immediate collision

            if dist < min_dist:
                min_dist = dist

    return min_dist

# ---- 4. CONFIGURATION CLASS ----

class Config:
    """Class to hold all configuration parameters."""
    def __init__(self):
        self.dt = DT
        self.predict_time = PREDICT_TIME
        self.robot_radius = ROBOT_RADIUS

        # Kinematic limits
        self.max_speed = MAX_SPEED
        self.max_yawrate = MAX_YAWRATE
        self.max_accel = MAX_ACCEL
        self.max_d_yawrate = MAX_D_YAWRATE

        # Sampling resolution
        self.v_resolution = V_RESOLUTION
        self.yaw_resolution = YAW_RESOLUTION

        # Weights
        self.weight_heading = WEIGHT_HEADING
        self.weight_distance = WEIGHT_DISTANCE
        self.weight_velocity = WEIGHT_VELOCITY
        self.weight_brake = WEIGHT_BRAKE
        self.min_speed = 0.02   # NEW: prevents DWA from choosing pure rotation

# ---- 5. VISUALIZATION AND MAIN EXECUTION (Optional for Lab) ----

def plot_robot(x, best_traj, ob, config):
    """Plots the current state, obstacles, and the best planned trajectory."""

    # Clear plot for animation effect
    plt.cla()

    # Plot obstacles
    for i in range(len(ob)):
        plt.plot(ob[i][0], ob[i][1], "ko", markersize=10)

    # Plot best trajectory
    plt.plot(best_traj[:, 0], best_traj[:, 1], "-g")

    # Plot robot position and orientation
    # Robot circle
    plt.gcf().gca().add_artist(Circle(xy=(x[0], x[1]), radius=config.robot_radius, fc='b', ec='k', alpha=0.5))
    # Robot heading line
    plt.plot([x[0], x[0] + math.cos(x[2]) * config.robot_radius],
             [x[1], x[1] + math.sin(x[2]) * config.robot_radius], "-k")

    plt.xlim(x[0] - 5, x[0] + 5)
    plt.ylim(x[1] - 5, x[1] + 5)
    plt.pause(0.001)

def main():
    """Simple simulation demonstrating DWA."""
    config = Config()

    # Initial State: [x, y, yaw, v, w]
    x = np.array([0.0, 0.0, math.pi / 2.0, 0.0, 0.0])

    # Goal Position: [x, y]
    goal = np.array([10.0, 10.0])

    # Obstacles: [[x, y], [x, y], ...]
    ob = np.array([
        [5.0, 5.0],
        [3.0, 8.0],
        [8.0, 3.0]
    ])

    # Main simulation loop
    plt.figure(figsize=(8, 8))
    plt.title("DWA Local Planning Simulation")
    plt.grid(True)

    while math.hypot(x[0] - goal[0], x[1] - goal[1]) > config.robot_radius:
        # 1. Run DWA
        u, best_traj = dwa_planning(x, goal, ob, config)

        # 2. Apply the chosen control (u) to the robot state for the next time step
        # This is the actual motion execution step
        x[3] = u[0]  # New velocity
        x[4] = u[1]  # New angular velocity

        x = x + np.array([
            x[3] * math.cos(x[2]) * config.dt,
            x[3] * math.sin(x[2]) * config.dt,
            x[4] * config.dt,
            0.0,
            0.0
        ])
        x[2] = normalize_angle(x[2])

        # 3. Visualization
        plot_robot(x, best_traj, ob, config)

    print("\nSimulation Complete: Goal Reached!")
    plt.show()

if __name__ == '__main__':
    main()
