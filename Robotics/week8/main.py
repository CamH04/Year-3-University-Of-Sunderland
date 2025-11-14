# main.py
# Main simulation runner for Hybrid Path Planning

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

import a_star_planner
import dwa_planner
from hybrid_planner import HybridPlanner, euclidean_distance

# ---- Visualization Functions ----

def plot_environment(ax, grid_map, start_rc, goal_rc, obstacles):
    """Plot the static grid map, start, goal, and dynamic obstacles."""
    ax.clear()

    # Plot grid
    rows, cols = grid_map.shape
    visual_map = np.copy(grid_map).astype(float)

    # Color mapping: 0=black (obstacle), 1=white (free), 5=gray (slow)
    cmap = plt.cm.colors.ListedColormap(['black', 'white', 'lightgray'])
    bounds = [0, 0.5, 3, 6]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    ax.imshow(visual_map, cmap=cmap, norm=norm, origin='upper', extent=[0, cols, rows, 0])

    # Draw grid lines
    for i in range(rows + 1):
        ax.axhline(i, color='gray', linewidth=0.5, alpha=0.3)
    for j in range(cols + 1):
        ax.axvline(j, color='gray', linewidth=0.5, alpha=0.3)

    # Plot start and goal
    ax.plot(start_rc[1] + 0.5, start_rc[0] + 0.5, 'go', markersize=15, label='Start', markeredgecolor='black', markeredgewidth=2)
    ax.plot(goal_rc[1] + 0.5, goal_rc[0] + 0.5, 'r*', markersize=20, label='Goal', markeredgecolor='black', markeredgewidth=1.5)

    # Plot dynamic obstacles
    for obs in obstacles:
        ax.plot(obs[0], obs[1], 'ko', markersize=12, label='Obstacle' if obs is obstacles[0] else '')

    ax.set_xlim(0, cols)
    ax.set_ylim(rows, 0)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.set_title('Hybrid Path Planning: A* + DWA')
    ax.set_xlabel('X (columns)')
    ax.set_ylabel('Y (rows)')

def plot_global_path(ax, global_path):
    """Plot the A* global path."""
    if global_path is not None:
        path_x = [pos[1] + 0.5 for pos in global_path]
        path_y = [pos[0] + 0.5 for pos in global_path]
        ax.plot(path_x, path_y, 'b--', linewidth=2, alpha=0.5, label='Global Path (A*)')

def plot_robot(ax, state, config, trajectory=None):
    """Plot the robot's current position, orientation, and predicted trajectory."""
    x, y, yaw = state[0], state[1], state[2]

    # Robot body (circle)
    robot_circle = Circle((x, y), config.robot_radius, fc='cyan', ec='blue', alpha=0.7, linewidth=2)
    ax.add_patch(robot_circle)

    # Robot heading (arrow)
    arrow_length = config.robot_radius * 1.5
    ax.arrow(x, y,
             arrow_length * math.cos(yaw),
             arrow_length * math.sin(yaw),
             head_width=0.3, head_length=0.2, fc='blue', ec='blue')

    # Plot DWA predicted trajectory
    if trajectory is not None and len(trajectory) > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], 'g-', linewidth=1.5, alpha=0.6, label='DWA Trajectory')

def plot_robot_path(ax, path_history):
    """Plot the historical path taken by the robot."""
    if len(path_history) > 1:
        path_array = np.array(path_history)
        ax.plot(path_array[:, 0], path_array[:, 1], 'r-', linewidth=2, alpha=0.8, label='Robot Path')

# ---- Main Simulation ----

def run_simulation(visualize=True, max_time=50.0):
    """
    Run the hybrid planner simulation.

    Args:
        visualize: Whether to show real-time visualization
        max_time: Maximum simulation time in seconds
    """

    print("=" * 60)
    print("HYBRID PATH PLANNING SIMULATION")
    print("=" * 60)

    # ---- Setup ----
    MAP = a_star_planner.GRID
    START_RC = a_star_planner.START
    GOAL_RC = a_star_planner.GOAL

    print(f"\nEnvironment: {MAP.shape[0]}x{MAP.shape[1]} grid")
    print(f"Start: {START_RC} (row, col)")
    print(f"Goal: {GOAL_RC} (row, col)")

    # Initial robot state [x, y, yaw, v, w]
    initial_state = np.array([START_RC[1] + 0.5, START_RC[0] + 0.5, math.pi / 2.0, 0.0, 0.0])

    # Dynamic obstacles [x, y]
    dynamic_obstacles = np.array([
        [5.0, 3.0],
        [7.0, 2.5]
    ])

    # ---- Initialize Planner ----
    try:
        planner = HybridPlanner(START_RC, GOAL_RC, MAP)
    except Exception as e:
        print(f"\n❌ Initialization failed: {e}")
        return None

    # ---- Simulation Variables ----
    current_state = initial_state.copy()
    sim_time = 0.0
    path_history = [current_state[:2].copy()]

    # Goal position in world coordinates
    goal_xy = (GOAL_RC[1] + 0.5, GOAL_RC[0] + 0.5)

    # ---- Visualization Setup ----
    if visualize:
        plt.ion()
        fig, ax = plt.subplots(figsize=(12, 10))

    # ---- Simulation Loop ----
    print("\n" + "=" * 60)
    print("STARTING SIMULATION")
    print("=" * 60)

    step_count = 0

    while sim_time < max_time:
        # Check if goal reached
        dist_to_goal = euclidean_distance(current_state[:2], goal_xy)
        if dist_to_goal <= planner.config.robot_radius:
            print(f"\n✅ Goal reached at t={sim_time:.1f}s!")
            break

        # Optional: Dynamic obstacle movement (uncomment to enable)
        # dynamic_obstacles[0, 1] = 3.0 + 1.0 * math.sin(sim_time * 0.5)

        # 1. Plan next step
        v, omega = planner.plan_step(current_state, None, dynamic_obstacles)

        # 2. Update robot state (kinematic model)
        dt = planner.config.dt
        current_state[0] += v * math.cos(current_state[2]) * dt
        current_state[1] += v * math.sin(current_state[2]) * dt
        current_state[2] += omega * dt
        current_state[2] = dwa_planner.normalize_angle(current_state[2])
        current_state[3] = v
        current_state[4] = omega

        # Store path
        path_history.append(current_state[:2].copy())

        # 3. Visualization
        if visualize and step_count % 5 == 0:  # Update every 5 steps for smoother animation
            plot_environment(ax, MAP, START_RC, GOAL_RC, dynamic_obstacles)
            plot_global_path(ax, planner.global_path)
            plot_robot_path(ax, path_history)
            plot_robot(ax, current_state, planner.config)

            # Add status text
            status_text = f"Time: {sim_time:.1f}s | WP: {planner.waypoint_index}/{len(planner.global_path)-1} | "
            status_text += f"Pos: ({current_state[0]:.2f}, {current_state[1]:.2f}) | "
            status_text += f"v={v:.2f} m/s, ω={omega:.2f} rad/s"
            ax.text(0.5, 1.02, status_text, transform=ax.transAxes,
                   ha='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

            plt.pause(0.01)

        # 4. Console output
        if step_count % 10 == 0:
            print(f"t={sim_time:5.1f}s | WP:{planner.waypoint_index:2d} | "
                  f"Pos:({current_state[0]:5.2f},{current_state[1]:5.2f}) | "
                  f"v={v:4.2f} | ω={omega:5.2f} | dist={dist_to_goal:5.2f}")

        sim_time += dt
        step_count += 1

    # ---- Final Summary ----
    print("\n" + "=" * 60)
    print("SIMULATION SUMMARY")
    print("=" * 60)
    print(f"Final Position: ({current_state[0]:.2f}, {current_state[1]:.2f})")
    print(f"Goal Position: {goal_xy}")
    print(f"Final Distance to Goal: {dist_to_goal:.2f} m")
    print(f"Total Simulation Time: {sim_time:.1f} s")
    print(f"Total Steps: {step_count}")
    print(f"Waypoints Reached: {planner.waypoint_index}/{len(planner.global_path)-1}")

    # Calculate path length
    total_distance = 0.0
    for i in range(1, len(path_history)):
        total_distance += euclidean_distance(path_history[i], path_history[i-1])
    print(f"Total Distance Traveled: {total_distance:.2f} m")

    if dist_to_goal <= planner.config.robot_radius:
        print("\n✅ SUCCESS: Robot reached the goal!")
    else:
        print("\n⚠️  TIMEOUT: Simulation ended before reaching goal.")

    # Final visualization
    if visualize:
        plot_environment(ax, MAP, START_RC, GOAL_RC, dynamic_obstacles)
        plot_global_path(ax, planner.global_path)
        plot_robot_path(ax, path_history)
        plot_robot(ax, current_state, planner.config)
        ax.text(0.5, 1.02, "SIMULATION COMPLETE", transform=ax.transAxes,
               ha='center', fontsize=12, fontweight='bold',
               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.9))
        plt.ioff()
        plt.show()

    return {
        'success': dist_to_goal <= planner.config.robot_radius,
        'final_position': current_state[:2],
        'distance_traveled': total_distance,
        'time_elapsed': sim_time,
        'path_history': path_history
    }

# ---- Entry Point ----

if __name__ == '__main__':
    # Run simulation with visualization
    results = run_simulation(visualize=True, max_time=50.0)

    if results:
        print("\n" + "=" * 60)
        print("Thank you for using the Hybrid Path Planner!")
        print("=" * 60)
