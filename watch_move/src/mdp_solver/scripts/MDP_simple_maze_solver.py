#######################
# Important Libraries #
#######################

import numpy as np
# import matplotlib.pyplot as plt
# from PIL import Image



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


"""
img = Image.open("path/to/your/file.png").convert('L')
arr = np.array(img)
# black (< threshold) → 1; white (≥ threshold) → 0
maze = (arr < 150).astype(int)

self.n_rows, n_cols = maze.shape
start = (0, 0)                # In this case, the robot starts at the top left corner
goal  = (0, n_cols-1)         # In this case, the robot starts at the top right corner

print(maze)
"""


#################
# MDP Algorithm #
#################

# Define the MDP class
class MDP:
    def __init__(self, maze, start, goal, gamma=0.99, r_step=-1, r_goal=1000):
        self.maze = maze
        self.start = start
        self.goal = goal
        self.actions = [(-1,0),(1,0),(0,-1),(0,1)]  # up,down,left,right
        self.gamma = gamma
        self.r_step = r_step
        self.r_goal = r_goal
        self.V = np.zeros_like(maze, dtype=float)          # Value Function
        self.policy = np.zeros((maze.shape[0], maze.shape[1]), dtype=int) # Policy
        self.n_rows, self.n_cols = maze.shape

    def in_bounds(self, row, col):
        return 0 <= row < self.n_rows and 0 <= col < self.n_cols

    def is_wall(self, row, col):
        return self.maze[row, col] == 1

    def is_goal(self, row, col):
        return (row, col) == self.goal
    
    def Val_Pol(self, iterations=1000):
        for i in range(iterations):
            delta = 0.0
            next_V = self.V.copy()
            for row in range(self.n_rows):
                for col in range(self.n_cols):

                    # Do nothing if there is a wall
                    if self.is_wall(row, col):
                        continue

                    # End the algorithm
                    if self.is_goal(row, col):
                        self.V[row, col] = self.r_goal
                        continue

                    best_val = -1e9
                    for a_idx, (dr, dc) in enumerate(self.actions):

                        nr, nc = row + dr, col + dc
                        if not self.in_bounds(nr, nc) or self.is_wall(nr, nc):
                            val = self.r_step + self.gamma * self.V[row, col]
                        
                        elif self.is_goal(nr, nc):
                            val = self.r_goal 

                        else:
                            val = self.r_step + self.gamma * self.V[nr, nc]

                        if val > best_val:
                            best_val = val
                            self.policy[row, col] = a_idx

                    next_V[row, col] = best_val

                    delta = max(delta, abs(self.V[row, col] - best_val))

            self.V = next_V
            if delta < 1e-4:
                print(f"Converged in {i} iterations.")
                break

        return self.V, self.policy
    
    def get_optimal_path(self):
        self.V, self.policy = self.Val_Pol()

        path = [self.start]
        cur = self.start

        while cur != self.goal:
            a = self.policy[cur[0], cur[1]]
            dr, dc = self.actions[a]
            nxt = (cur[0] + dr, cur[1] + dc)
            if not self.in_bounds(*nxt) or self.is_wall(*nxt):
                print("Stuck or wall encountered—check your maze definition!")
                break
            
            path.append(nxt)
            cur = nxt

        return path
    
    def optimal_path_plot(self, path):
        fig, ax = plt.subplots(figsize=(6,4))
        # draw maze
        ax.imshow(self.maze, cmap='gray_r')
        # overlay path
        ys, xs = zip(*path)
        ax.plot(xs, ys, '-o', linewidth=2)
        ax.scatter([self.start[1],self.goal[1]], [self.start[0],self.goal[0]],
                   c=['green','red'], s=100, label='start/goal')
        ax.set_title("Optimal Path in Maze")
        ax.set_xticks(range(self.n_cols)); ax.set_yticks(range(self.n_rows))
        ax.set_xlim(-0.5, self.n_cols-0.5); ax.set_ylim(self.n_rows-0.5, -0.5)
        ax.grid(True)
        plt.legend(); plt.show()

    def value_function_plot(self):
        fig, ax = plt.subplots(figsize=(9,6))
        # Draw heatmap
        cmap = plt.cm.viridis
        # Mask walls so they appear white
        V_masked = np.ma.masked_where(self.maze==1, self.V)
        heat = ax.imshow(V_masked, cmap=cmap, interpolation='nearest')


        # Grid lines
        ax.set_xticks(np.arange(-.5, self.n_cols, 1), minor=True)
        ax.set_yticks(np.arange(-.5, self.n_rows, 1), minor=True)
        ax.grid(which='minor', color='w', linewidth=2)

        # Ticks off
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title("Value Function")

        plt.tight_layout()
        plt.show()



###############
# Run the MDP #
###############      

mdp = MDP(maze, start, goal)
V, policy = mdp.Val_Pol()
path = mdp.get_optimal_path()
mdp.optimal_path_plot(path)
mdp.value_function_plot()

