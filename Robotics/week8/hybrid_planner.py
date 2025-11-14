# hybrid_planner.py

import numpy as np
import math

import a_star_planner
import dwa_planner


def euclidean_distance(pos1, pos2):
    """Calculates the 2D Euclidean distance between two points (r, c) or (x, y)."""
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

class HybridPlanner:
    def __init__(self, start_pos, goal_pos, grid_map):
        print("Initializing Global Planner (A*)...")
        self.global_path, _ = a_star_planner.a_star(grid_map, start_pos, goal_pos)

        if self.global_path is None:
            raise Exception("A* failed to find a global path.")

        print(f"Global Path Found with {len(self.global_path)} waypoints.")

        self.waypoint_index = 0
        self.DWA_TARGET_DISTANCE = 1.0

        self.config = dwa_planner.Config()
    def determine_next_waypoint(self, current_state):
        if self.global_path is None or self.waypoint_index >= len(self.global_path):
            return None

        # Get the current target waypoint position (r, c) or (x, y)
        target_waypoint_pos = self.global_path[self.waypoint_index]
        waypoint_xy = (target_waypoint_pos[1] + 0.5, target_waypoint_pos[0] + 0.5)

        distance = euclidean_distance(current_state[:2], waypoint_xy)

        if distance < self.DWA_TARGET_DISTANCE and self.waypoint_index < len(self.global_path) - 1:
            self.waypoint_index += 1
            target_waypoint_pos = self.global_path[self.waypoint_index]
            waypoint_xy = (target_waypoint_pos[1] + 0.5, target_waypoint_pos[0] + 0.5)
        return waypoint_xy

    def plan_step(self, current_state, sensor_data, obstacles):
        local_target_pos = self.determine_next_waypoint(current_state)

        if local_target_pos is None:
            print("Final Goal Reached or No Path. Stopping.")
            return 0.0, 0.0
        local_target_dwa = np.array(local_target_pos)
        u_best, best_traj = dwa_planner.dwa_planning(current_state, local_target_dwa, obstacles, self.config)

        v, omega = u_best[0], u_best[1]
        return v, omega

if __name__ == '__main__':
    MAP = a_star_planner.GRID
    START_RC = a_star_planner.START
    GOAL_RC = a_star_planner.GOAL

    initial_state = np.array([START_RC[1] + 0.5, START_RC[0] + 0.5, math.pi / 2.0, 0.0, 0.0])

    dynamic_obstacles = np.array([
        [5.0, 3.0]
    ])
    try:
        planner = HybridPlanner(START_RC, GOAL_RC, MAP)
    except Exception as e:
        print(f"Initialization failed: {e}")
        exit()

    current_state = initial_state
    sim_time = 0.0

    while sim_time < 50.0 and euclidean_distance(current_state[:2], (GOAL_RC[1] + 0.5, GOAL_RC[0] + 0.5)) > planner.config.robot_radius:
        v, omega = planner.plan_step(current_state, None, dynamic_obstacles)
        dt = planner.config.dt
        current_state[0] += v * math.cos(current_state[2]) * dt
        current_state[1] += v * math.sin(current_state[2]) * dt
        current_state[2] += omega * dt
        current_state[2] = dwa_planner.normalize_angle(current_state[2])
        current_state[3] = v
        current_state[4] = omega

        print(f"Time: {sim_time:.1f}s | Target WP: {planner.waypoint_index} | Pos: ({current_state[0]:.2f}, {current_state[1]:.2f}) | Cmd: (v={v:.2f}, w={omega:.2f})")

        sim_time += dt

    print("\n---- Simulation Summary ----")
    print(f"Final Position: ({current_state[0]:.2f}, {current_state[1]:.2f})")
    print(f"Goal Position: ({GOAL_RC[1]+0.5}, {GOAL_RC[0]+0.5})")
    print(f"Simulation Ended at T={sim_time:.1f}s")
