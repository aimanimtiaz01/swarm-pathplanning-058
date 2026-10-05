import math
import random
from collections import deque
import matplotlib.pyplot as plt

# using my roll number 58 so the grid and obstacles stay exactly the same every time I run it
SEED = 58
GRID_SIZE = 20
OBSTACLE_DENSITY = 0.20
NUM_WAYPOINTS = 6
SWARM_SIZE = 60
MAX_ITERATIONS = 200

W_START = 0.9
W_END = 0.4
C1 = 1.7
C2 = 1.7

# massive penalty to force the swarm to prioritize avoiding obstacles before finding the shortest path
COLLISION_PENALTY = 100.0

random.seed(SEED)


def is_reachable(obstacles, start, goal):
# simple BFS check to make sure a path actually exists, otherwise the PSO will just run infinitely without finding a valid route
    queue = deque([start])
    seen = {start}
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                    if (nx, ny) not in obstacles and (nx, ny) not in seen:
                        seen.add((nx, ny))
                        queue.append((nx, ny))
    return False

def generate_problem():
    while True:
        all_cells = [(x, y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)]
        num_blocked = int(GRID_SIZE * GRID_SIZE * OBSTACLE_DENSITY)
        obstacles = set(random.sample(all_cells, num_blocked))
        
        free_cells = [c for c in all_cells if c not in obstacles]
        start, goal = random.sample(free_cells, 2)
        
# making sure start and goal aren't placed right next to each other so the problem remains challenging        
        far_enough = math.dist(start, goal) >= GRID_SIZE * 0.6
        if far_enough and is_reachable(obstacles, start, goal):
            return obstacles, start, goal