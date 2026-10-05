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



def decode_particle(position, start, goal):
    waypoints = []
    for i in range(0, len(position), 2):

# taking the flat array of particle numbers and grouping them into proper (x, y) coordinates for the grid
        waypoints.append((position[i], position[i+1]))
    return [start] + waypoints + [goal]

def count_collisions_sampling(path_points, obstacles):
    hits = 0
    for i in range(len(path_points) - 1):
        x1, y1 = path_points[i]
        x2, y2 = path_points[i+1]
        dist = math.dist((x1, y1), (x2, y2))

# breaking the line segment into tiny mathematical steps to check if any of those points fall inside a blocked obstacle cell        
        steps = max(2, int(dist * 4)) 
        for step in range(steps + 1):
            t = step / steps
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            
            cell_x, cell_y = int(round(px)), int(round(py))
            if (cell_x, cell_y) in obstacles:
                hits += 1
    return hits

def path_length(path_points):
    return sum(math.dist(path_points[i], path_points[i+1]) for i in range(len(path_points)-1))

def fitness(position, start, goal, obstacles):
    path = decode_particle(position, start, goal)

# fitness score: real length of the path plus the heavy penalty if it hits anything. Lower is better.    
    return path_length(path) + COLLISION_PENALTY * count_collisions_sampling(path, obstacles)