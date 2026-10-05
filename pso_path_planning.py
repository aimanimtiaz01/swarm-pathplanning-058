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


def run_pso(start, goal, obstacles):
    dim = NUM_WAYPOINTS * 2
    lower, upper = 0.0, GRID_SIZE - 1.0
    v_max = 0.2 * (upper - lower)

    positions, velocities, pbest_pos, pbest_cost = [], [], [], []
    
    for _ in range(SWARM_SIZE):
        pos = [random.uniform(lower, upper) for _ in range(dim)]
        vel = [random.uniform(-v_max, v_max) for _ in range(dim)]
        positions.append(pos)
        velocities.append(vel)
        pbest_pos.append(pos[:])
        pbest_cost.append(fitness(pos, start, goal, obstacles))

    best_i = min(range(SWARM_SIZE), key=lambda i: pbest_cost[i])
    gbest_pos = pbest_pos[best_i][:]
    gbest_cost = pbest_cost[best_i]
    
    history = [gbest_cost]

    for it in range(MAX_ITERATIONS):
        # # slowly decreasing inertia over time so particles explore the grid first, then settle down to exploit the best path at the end
        w = W_START - (W_START - W_END) * (it / (MAX_ITERATIONS - 1))
        for i in range(SWARM_SIZE):
            for d in range(dim):
                r1, r2 = random.random(), random.random()

                # the main PSO formula: 'w' keeps its momentum, 'c1' pulls it towards its personal best, and 'c2' pulls it towards the swarm's global best
                velocities[i][d] = (w * velocities[i][d] + 
                                    C1 * r1 * (pbest_pos[i][d] - positions[i][d]) + 
                                    C2 * r2 * (gbest_pos[d] - positions[i][d]))
                
                velocities[i][d] = max(-v_max, min(v_max, velocities[i][d]))

                # clipping the coordinates so the particles don't fly off our 20x20 grid boundaries
                positions[i][d] = max(lower, min(upper, positions[i][d] + velocities[i][d]))

            cost = fitness(positions[i], start, goal, obstacles)
            if cost < pbest_cost[i]:
                pbest_cost[i] = cost
                pbest_pos[i] = positions[i][:]
                if cost < gbest_cost:
                    gbest_cost = cost
                    gbest_pos = positions[i][:]
        history.append(gbest_cost)
                    
    return gbest_pos, gbest_cost, history



def plot_result(obstacles, start, goal, path, history):
    fig1, ax1 = plt.subplots(figsize=(7, 7))
    for (x, y) in obstacles:
        ax1.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="black"))
        
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax1.plot(xs, ys, color="royalblue", linewidth=2.5, label="PSO Path")
    ax1.scatter(xs[1:-1], ys[1:-1], color="orange", s=50, zorder=4, label="Waypoints")

    # explicitly marking the start and goal cells 
    ax1.scatter(*start, color="green", s=150, zorder=5, label="Start")
    ax1.scatter(*goal, color="red", s=150, zorder=5, marker="*", label="Goal")
    
    ax1.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax1.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax1.grid(True, color="lightgray", linestyle="--")
    ax1.legend(loc="upper right")
    ax1.set_title(f"PSO Path Planning (Seed {SEED})")
    fig1.savefig("pso_path.png")

    fig2, ax2 = plt.subplots(figsize=(6, 4))
    ax2.plot(history, color="purple", linewidth=2)
    ax2.set_xlabel("Iteration")
    ax2.set_ylabel("Best Cost")
    ax2.set_title("PSO Convergence")
    ax2.grid(True)
    fig2.savefig("pso_convergence.png")
    
    plt.show()

def main():
    obstacles, start, goal = generate_problem()
    print(f"Seed: {SEED} | Start: {start} | Goal: {goal}")
    
    best_pos, best_cost, history = run_pso(start, goal, obstacles)
    best_path = decode_particle(best_pos, start, goal)
    
    
    collisions = count_collisions_sampling(best_path, obstacles)
    length = path_length(best_path)
    
    print(f"\n--- RESULTS ---")
    print(f"Total Collisions: {collisions}")
    print(f"Path Length: {length:.3f}")
    print(f"Final Cost (Fitness): {best_cost:.3f}")
    
    if collisions == 0:
        print("Verdict: Obstacle-free path found!")
    else:
        print("Verdict: Path still has collisions.")
        
    plot_result(obstacles, start, goal, best_path, history)

if __name__ == "__main__":
    main()