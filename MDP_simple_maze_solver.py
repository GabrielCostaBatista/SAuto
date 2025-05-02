#######################
# Important Libraries #
#######################

import numpy as np
import matplotlib.pyplot as plt



###################
# Maze Definition #
###################

maze = np.array([
    [0,0,0,0,1,0,0,0,0],
    [1,1,1,0,1,0,1,1,1],
    [0,0,0,0,1,0,1,0,0],
    [0,1,0,1,1,0,1,1,0],
    [0,1,0,0,1,0,0,1,0],
    [0,0,1,0,0,0,0,0,0],
], dtype=int)

n_rows, n_cols = maze.shape
start = (0, 0)                # In this case, the robot starts at the top left corner
goal  = (0, n_cols-1)         # In this case, the robot starts at the top right corner



#################
# MDP Algorithm #
#################

actions = [(-1,0),(1,0),(0,-1),(0,1)]  # up,down,left,right
gamma = 0.99
r_step = -1
r_goal = 0

def in_bounds(row,col): # Function to make sure that the robot does not leave the maze
    return 0 <= row < n_rows and 0 <= col < n_cols

V = np.zeros_like(maze, dtype=float)          # Value Function
policy = np.zeros((n_rows,n_cols), dtype=int) # Policy

# Calculate the Value Function and the Policy
for i in range(1000):
    delta = 0.0
    V_prev = V.copy()
    for row in range(n_rows):
        for col in range(n_cols):

            # Do nothing if there is a wall
            if maze[row,col]==1:
                continue

            # End the algorithm
            if (row,col)==goal:
                V[row,col] = r_goal
                continue

            best_val = -1e9
            for a_idx,(dr,dc) in enumerate(actions):

                nr, nc = row+dr, col+dc
                if not in_bounds(nr,nc) or maze[nr,nc]==1:
                    val = r_step + gamma * V_prev[row,col]

                else:
                    val = r_step + gamma * V_prev[nr,nc]

                if val > best_val:
                    best_val = val
                    policy[row,col] = a_idx

            V[row,col] = best_val

            delta = max(delta, abs(V[row,col] - V_prev[row,col]))

    if delta < 1e-4:
        print(f"Converged in {i} iterations.")
        break

path = [start]
cur = start

while cur != goal:
    a = policy[cur[0], cur[1]]
    dr, dc = actions[a]
    nxt = (cur[0]+dr, cur[1]+dc)
    if not in_bounds(*nxt) or maze[nxt]==1:
        print("Stuck or wall encountered—check your maze definition!")
        break
    
    path.append(nxt)
    cur = nxt

print("Optimal path:")
print(path)



##################
# Make the Plots #
##################

# Show the optimal path
fig, ax = plt.subplots(figsize=(6,4))
# draw maze
ax.imshow(maze, cmap='gray_r')
# overlay path
ys, xs = zip(*path)
ax.plot(xs, ys, '-o', linewidth=2)
ax.scatter([start[1],goal[1]], [start[0],goal[0]],
           c=['green','red'], s=100, label='start/goal')
ax.set_title("Optimal Path in Maze")
ax.set_xticks(range(n_cols)); ax.set_yticks(range(n_rows))
ax.set_xlim(-0.5, n_cols-0.5); ax.set_ylim(n_rows-0.5, -0.5)
ax.grid(True)
plt.legend(); plt.show()

# Show the Value Function map
fig, ax = plt.subplots(figsize=(9,6))
# Draw heatmap
cmap = plt.cm.viridis
# Mask walls so they appear white
V_masked = np.ma.masked_where(maze==1, -V)
heat = ax.imshow(-V_masked, cmap=cmap, interpolation='nearest')

# Annotate each cell
for r in range(n_rows):
    for c in range(n_cols):
        if maze[r,c] == 1:
            continue
        text = ""
        if (r,c) == start:
            text = "S"
        elif (r,c) == goal:
            text = "G"
        else:
            text = f"{-V[r,c]:.0f}"
        ax.text(c, r, text, ha='center', va='center', color='white', fontsize=12)


# Grid lines
ax.set_xticks(np.arange(-.5, n_cols, 1), minor=True)
ax.set_yticks(np.arange(-.5, n_rows, 1), minor=True)
ax.grid(which='minor', color='w', linewidth=2)

# Ticks off
ax.set_xticks([])
ax.set_yticks([])
ax.set_title("Value Function")

plt.tight_layout()
plt.show()