#######################
# Important Libraries #
#######################

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from PIL import Image

# Maze: 0=free,1=wall,2=marker
maze = np.array([
    [0,0,2,0,1,0,0,0,0],
    [1,1,1,0,1,2,1,1,1],
    [0,0,0,0,1,0,1,0,0],
    [0,1,0,1,1,0,1,1,2],
    [0,1,2,0,1,0,0,1,0],
    [0,0,1,0,0,0,0,0,0],
], dtype=int)

img = Image.open("maze_with_markers.png").convert('L')
arr = np.array(img)

# define your thresholds
th_black = 20
th_gray  = 200

maze = np.zeros_like(arr, dtype=int)

maze[arr < th_black] = 1

mask_gray = (arr >= th_black) & (arr < th_gray)
maze[mask_gray] = 2


class POMDP:
    def __init__(self, maze, start, goal,
                 gamma=0.95, r_step=-1, r_goal=1000, slip=0.2):
        
        self.maze = maze
        self.start = start
        self.goal = goal
        self.gamma = gamma
        self.r_step = r_step
        self.r_goal = r_goal
        self.slip = slip

        # Actions and states
        self.actions = [(-1,0),(1,0),(0,-1),(0,1),(0,0)]  # up,down,left,right,observe
        self.states = [(x,y) for x in range(maze.shape[0])
                                for y in range(maze.shape[1])
                                if maze[x,y] in (0,2)]
        
        self.S = len(self.states)
        self.A = len(self.actions)
        self.state_index = {s:i for i,s in enumerate(self.states)}

        # Precompute transition model T[a]
        self.T = np.zeros((self.A, self.S, self.S))
        slip_map = {(-1,0):(0,1),(0,1):(1,0),(1,0):(0,-1),(0,-1):(-1,0)}
        for a_idx, action in enumerate(self.actions):
            for s_idx, s in enumerate(self.states):
                if action==(0,0):
                    self.T[a_idx,s_idx,s_idx]=1.0
                else:
                    intended = self.move(s, action)
                    slipped  = self.move(s, slip_map[action])
                    i_int = self.state_index[intended]
                    i_slp = self.state_index[slipped]
                    self.T[a_idx,s_idx,i_int] += 1-self.slip
                    self.T[a_idx,s_idx,i_slp] += self.slip

        # Precompute observation model O
        self.O = np.array([1.0 if maze[s]==2 else 0.0 for s in self.states])

        # Reward model R[a,s] = expected reward under slip
        self.R = np.zeros((self.A, self.S))
        for a_idx, action in enumerate(self.actions):
            for s_idx, s in enumerate(self.states):
                if action==(0,0):
                    self.R[a_idx,s_idx]=0.0
                else:
                    intended = self.move(s, action)
                    slipped  = self.move(s, slip_map[action])
                    def cell_r(c):
                        if c==self.goal: return self.r_goal
                        if maze[c]==2:   return 2.0
                        return self.r_step
                    r_int = cell_r(intended)
                    r_slp = cell_r(slipped)
                    self.R[a_idx,s_idx] = (1-self.slip)*r_int + self.slip*r_slp

    def move(self, s, action):
        x,y = s; dx,dy = action
        nx,ny = np.clip(x+dx,0,self.maze.shape[0]-1), np.clip(y+dy,0,self.maze.shape[1]-1)
        return (nx,ny) if self.maze[nx,ny]!=1 else s

    def update_belief(self, b, a_idx, obs):
        b_bar = self.T[a_idx].T.dot(b)
        if obs=="marker":
            b_bar *= self.O
        else:
            b_bar *= (1-self.O)
        total = b_bar.sum()
        return b_bar/total if total>0 else np.ones(self.S)/self.S

    def reward(self, b, a_idx):
        # expected immediate reward under belief b and action a
        return b.dot(self.R[a_idx])
    
    def solve_mdp(self, tol=1e-3):
        # initialise V(s)=0
        V = np.zeros(self.S)
        while True:
            delta = 0.0
            for s_idx in range(self.S):
                v_old = V[s_idx]
                # Bellman backup: max over actions
                V[s_idx] = max(
                    self.R[a_idx, s_idx]
                    + self.gamma * (self.T[a_idx][s_idx].dot(V))
                    for a_idx in range(self.A)
                )
                delta = max(delta, abs(V[s_idx] - v_old))
            if delta < tol:
                break
        # store V
        self.V = V
        # compute Q[a,s]
        self.Q = np.zeros((self.A, self.S))
        for a_idx in range(self.A):
            # vectorised: R[a] + γ * T[a] @ V
            self.Q[a_idx] = self.R[a_idx] + self.gamma * (self.T[a_idx].dot(V))

    def qmdp_action(self, belief):
        # expected Q for each action: sum_s b(s) * Q[a,s]
        exp_Q = belief.dot(self.Q.T)
        return int(np.argmax(exp_Q))

start = (0, 0)
goal = (0, 26)

pomdp = POMDP(maze, start, goal,
              gamma=0.95, r_step=-1, r_goal=1000, slip=0.2)

# one-shot QMDP solve
pomdp.solve_mdp(tol=1e-4)

# initial belief
b = np.zeros(pomdp.S)
b[pomdp.state_index[start]] = 1.0

true = start
path = [true]

for t in range(1000000):
    # 1) pick action by QMDP policy
    a_idx = pomdp.qmdp_action(b)

    # 2) simulate true next state
    probs = pomdp.T[a_idx, pomdp.state_index[true]]
    true = pomdp.states[np.random.choice(pomdp.S, p=probs)]

    # 3) simulate observation
    obs = "marker" if (np.random.rand() < pomdp.O[pomdp.state_index[true]]) else None

    if obs is None:
        b = pomdp.update_belief(b, a_idx, obs)
    else:
        b = pomdp.update_belief(b, a_idx, obs)
        b[pomdp.state_index[true]] = 1.0


    path.append(true)
    if true == goal or b[pomdp.state_index[goal]] > 0.99:
        print(f"Goal reached at step {t+1}")
        break

##########################
# Testing Implementation #
##########################

cmap = ListedColormap(['white', 'black', 'gray'])

n_rows, n_cols = maze.shape


fig, ax = plt.subplots(figsize=(6, 4))

cmap = ListedColormap(['white', 'black', 'gray'])
ax.imshow(maze, cmap=cmap, vmin=0, vmax=2, origin='upper')

# 4) Overlay the path in blue with circle markers
ys, xs = zip(*path)
ax.plot(xs, ys, '-o', linewidth=2, label='path')

# 5) Mark start (green) and goal (red)
ax.scatter([start[1], goal[1]], [start[0], goal[0]],
           c=['green', 'red'], s=100, label='start/goal')

# 6) Formatting
ax.set_title("Optimal Path in Maze")
ax.set_xticks(range(n_cols))
ax.set_yticks(range(n_rows))
ax.set_xlim(-0.5, n_cols - 0.5)
ax.set_ylim(n_rows - 0.5, -0.5)
ax.grid(True)
plt.legend()
plt.tight_layout()
plt.show()