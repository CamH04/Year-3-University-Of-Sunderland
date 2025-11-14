import numpy as np
import heapq
import matplotlib.pyplot as plt

GRID = np.array([
    [1, 1, 1, 1, 1, 5, 1, 1, 1, 1],
    [1, 5, 5, 5, 1, 5, 1, 0, 1, 1],
    [1, 1, 1, 1, 1, 5, 1, 0, 1, 1],
    [1, 0, 0, 0, 0, 1, 1, 0, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
])

START = (0, 0)  # (row, col)
GOAL = (4, 9)

MOVEMENTS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
ROWS, COLS = GRID.shape

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

        # g: Cost from start to current node
        self.g = float('inf')
        # h: Heuristic (estimated cost to goal)
        self.h = 0
        # f: Total cost (g + h)
        self.f = float('inf')

    def __lt__(self, other):
        if self.f == other.f:
            return self.g < other.g
        return self.f < other.f

    def __eq__(self, other):
        """Used to check if two nodes are at the same position."""
        return self.position == other.position

    def __hash__(self):
        """Allows Node objects to be stored in sets/dictionaries."""
        return hash(self.position)

def heuristic(pos1, pos2):
    """
    Calculates the Manhattan distance (L1 norm) between two points.
    h = |r1 - r2| + |c1 - c2|
    """
    r1, c1 = pos1
    r2, c2 = pos2
    return abs(r1 - r2) + abs(c1 - c2)

def a_star(grid, start, goal):
    if grid[start] == 0 or grid[goal] == 0:
        print("Start or Goal is an obstacle.")
        return None, []

    start_node = Node(start)
    start_node.g = 0
    start_node.h = heuristic(start, goal)
    start_node.f = start_node.g + start_node.h

    open_list = [(start_node.f, start_node)]

    closed_list = {start: start_node}

    expanded_nodes = []
    while open_list:
        current_f, current_node = heapq.heappop(open_list)
        expanded_nodes.append(current_node.position)

        if current_node.position == goal:
            path = []
            curr = current_node
            while curr is not None:
                path.append(curr.position)
                curr = curr.parent
            return path[::-1], expanded_nodes

        for dr, dc in MOVEMENTS:
            neighbor_pos = (current_node.position[0] + dr, current_node.position[1] + dc)

            if not (0 <= neighbor_pos[0] < ROWS and 0 <= neighbor_pos[1] < COLS):
                continue
            if grid[neighbor_pos] == 0:
                continue

            movement_cost = grid[neighbor_pos]
            new_g = current_node.g + movement_cost

            if neighbor_pos in closed_list:
                neighbor_node = closed_list[neighbor_pos]
            else:
                neighbor_node = Node(neighbor_pos)
                neighbor_node.h = heuristic(neighbor_pos, goal)
                closed_list[neighbor_pos] = neighbor_node

            if new_g < neighbor_node.g:
                neighbor_node.g = new_g
                neighbor_node.f = neighbor_node.g + neighbor_node.h
                neighbor_node.parent = current_node

                heapq.heappush(open_list, (neighbor_node.f, neighbor_node))

    return None, expanded_nodes

def visualize_path(grid, path, expanded_nodes):

    visual_map = np.copy(grid)

    for r, c in expanded_nodes:
        if (r, c) != START and (r, c) != GOAL:
            visual_map[r, c] = 2.5
    if path:
        for r, c in path:
            if (r, c) != START and (r, c) != GOAL:
                visual_map[r, c] = 10

    plt.figure(figsize=(10, 5))

    cmap = plt.cm.get_cmap('viridis', 6)
    norm = plt.Normalize(vmin=0, vmax=11)

    plt.imshow(visual_map, cmap=cmap, norm=norm)
    plt.title("A* Path Planning Result: Cost-Aware Search")

    ax = plt.gca()
    ax.set_xticks(np.arange(-.5, COLS, 1), minor=True)
    ax.set_yticks(np.arange(-.5, ROWS, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)

    plt.plot(START[1], START[0], 'o', color='lime', markersize=10, label='Start')
    plt.plot(GOAL[1], GOAL[0], 'X', color='red', markersize=10, label='Goal')

    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', label='Start', markerfacecolor='lime', markersize=10),
        plt.Line2D([0], [0], marker='X', color='w', label='Goal', markerfacecolor='red', markersize=10),
        plt.Rectangle((0, 0), 1, 1, fc=cmap(norm(10)), label='Final Path (Lowest Cost)'),
        plt.Rectangle((0, 0), 1, 1, fc=cmap(norm(2.5)), label='Expanded Nodes (Search Space)'),
        plt.Rectangle((0, 0), 1, 1, fc=cmap(norm(5)), label='Slow Terrain (Cost 5)'),
        plt.Rectangle((0, 0), 1, 1, fc=cmap(norm(1)), label='Free Space (Cost 1)'),
        plt.Rectangle((0, 0), 1, 1, fc=cmap(norm(0)), label='Obstacle (Cost 0)')
    ]
    plt.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.4, 1.05))
    plt.show()


if __name__ == "__main__":
    print(f"Starting A* Path Planning from {START} to {GOAL}...")

    final_path, expanded = a_star(GRID, START, GOAL)

    if final_path:
        total_cost = sum(GRID[r][c] for r, c in final_path if (r, c) != START)

        print("\n✅ Path Found!")
        print(f"   Total Nodes Expanded: {len(expanded)}")
        print(f"   Total Path Cost: {total_cost}")
        print(f"   Path: {final_path}")
        visualize_path(GRID, final_path, expanded)
    else:
        print("\n❌ No path found.")
        visualize_path(GRID, [], expanded)
