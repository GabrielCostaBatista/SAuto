import numpy as np

class Maze:
    """
    Represents a 2D grid maze where 0 = free cell, 1 = wall.
    States are indices of free cells.
    Checkpoints are user-defined cells where the robot can localise perfectly.
    """
    def __init__(self, grid, start, goal, checkpoints=None):
        self.grid = np.array(grid)
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.checkpoints = [tuple(c) for c in (checkpoints or [])]
        self._idx_map = {}
        self._rev_map = {}
        idx = 0
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i, j] == 0:
                    self._idx_map[(i, j)] = idx
                    self._rev_map[idx] = (i, j)
                    idx += 1
        self.n_states       = idx
        self.start_idx      = self._idx_map[self.start]
        self.goal_idx       = self._idx_map[self.goal]
        self.checkpoint_idxs = [self._idx_map[c] for c in self.checkpoints]

    def state_to_coord(self, s):
        return self._rev_map[s]

    def coord_to_state(self, coord):
        return self._idx_map[tuple(coord)]

    def neighbors(self, coord):
        """
        Yields adjacent free-cell coordinates (up/down/left/right).
        """
        i, j = coord
        for di, dj in [(-1,0), (1,0), (0,-1), (0,1)]:
            ni, nj = i + di, j + dj
            if (0 <= ni < self.grid.shape[0] and
                0 <= nj < self.grid.shape[1] and
                self.grid[ni, nj] == 0):
                yield (ni, nj)

    def shortest_path(self, start, goal):
        """
        BFS to find the shortest path of coords from start→goal.
        Uses only numpy and Python lists (no deque).
        Returns list of (r,c). Empty if no path.
        """
        # frontier holds coords to explore
        frontier = [start]
        # map each visited coord to its predecessor
        prev = {start: None}

        while frontier:
            u = frontier.pop(0)   # pop from head
            if u == goal:
                break
            for v in self.neighbors(u):
                if v not in prev:
                    prev[v] = u
                    frontier.append(v)

        if goal not in prev:
            return []  # no path found

        # reconstruct path by walking backwards
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev[node]
        return path[::-1]

    def coords_to_actions(self, path):
        """
        Convert a coord path [(r0,c0),(r1,c1),...] to actions ['up',...].
        """
        acts = []
        for (r0,c0),(r1,c1) in zip(path, path[1:]):
            dr, dc = r1-r0, c1-c0
            if   (dr,dc)==(-1,0): acts.append('up')
            elif (dr,dc)==( 1,0): acts.append('down')
            elif (dr,dc)==( 0,1): acts.append('right')
            elif (dr,dc)==( 0,-1):acts.append('left')
        return acts

    def __repr__(self):
        return f"Maze(start={self.start}, goal={self.goal}, checkpoints={self.checkpoints})"

class MDP:
    """
    Computes optimal Q and V using value iteration for the goal-reaching task.
    Modified so that bumping into a wall redistributes motion to valid neighbors.
    """
    def __init__(self, maze, slip_prob=0.1, step_cost=-1, goal_reward=100, gamma=0.95):
        self.maze        = maze
        self.n           = maze.n_states
        self.actions     = ['up','down','left','right']
        self.slip        = slip_prob
        self.gamma       = gamma
        self.step_cost   = step_cost
        self.goal_reward = goal_reward

        self.P = np.zeros((self.n, len(self.actions), self.n))
        self.R = np.full((self.n, len(self.actions), self.n), self.step_cost)

        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        # Define which directions are "sides" for each action (exclude opposite)
        side_actions = {
            'up': ['left', 'right'],
            'down': ['left', 'right'], 
            'left': ['up', 'down'],
            'right': ['up', 'down']
        }
        
        for s in range(self.n):
            if s == maze.goal_idx:
                self.P[s,:,s] = 1.0
                continue
            ci = maze.state_to_coord(s)
            free_neigh = list(maze.neighbors(ci))
            for a_idx, a in enumerate(self.actions):
                sides = side_actions[a]
                for b in self.actions:
                    if b == a:
                        prob = 1 - self.slip  # intended direction
                    elif b in sides:
                        prob = self.slip / 2  # slip to sides only
                    else:
                        prob = 0  # no slip backwards or staying
                    
                    if prob > 0:
                        di, dj = directions[b]
                        ni, nj = ci[0]+di, ci[1]+dj
                        if (ni, nj) in maze._idx_map:
                            sp = maze.coord_to_state((ni, nj))
                            self.P[s,a_idx,sp] += prob
                            if sp == maze.goal_idx:
                                self.R[s,a_idx,sp] = self.goal_reward
                        else:
                            # redistribute to free neighbors if bump into wall
                            if free_neigh:
                                for nbr in free_neigh:
                                    spn = maze.coord_to_state(nbr)
                                    self.P[s,a_idx,spn] += prob/len(free_neigh)
                                    if spn == maze.goal_idx:
                                        self.R[s,a_idx,spn] = self.goal_reward
                            else:
                                # trapped: stay put
                                self.P[s,a_idx,s] += prob

        self.V = np.zeros(self.n)
        self.Q = np.zeros((self.n, len(self.actions)))

    def value_iteration(self, eps=1e-3):
        while True:
            delta = 0
            for s in range(self.n):
                if s == self.maze.goal_idx:
                    continue
                q_vals = np.sum(self.P[s] * (self.R[s] + self.gamma * self.V), axis=1)
                max_q  = np.max(q_vals)
                delta  = max(delta, abs(max_q - self.V[s]))
                self.V[s] = max_q
            if delta < eps:
                break
        for s in range(self.n):
            self.Q[s] = np.sum(self.P[s] * (self.R[s] + self.gamma * self.V), axis=1)



class QMDPController:
    """
    Controls a real robot with QMDP policy; uses checkpoint detections for relocalisation.
    """
    def __init__(self, mdp, entropy_thresh=0.9):
        self.mdp = mdp
        self.belief = None
        self.entropy_thresh = entropy_thresh

    def init_belief(self):
        b = np.zeros(self.mdp.n)
        b[self.mdp.maze.start_idx] = 1.0
        self.belief = b

    def belief_entropy(self):
        p = self.belief
        return -np.sum(p[p>0] * np.log(p[p>0]))

    def predict_belief(self, action_idx):
        b_pred = self.belief @ self.mdp.P[:,action_idx,:]
        b_pred = b_pred / b_pred.sum()
        
        # Add forward spreading in the direction of movement
        spreading_factor = 0.15  # Controls how much spreading to add
        action = self.mdp.actions[action_idx]
        
        # Create directional spreading kernel
        forward_spread = np.zeros_like(b_pred)
        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        dr, dc = directions[action]
        
        # For each state with significant belief, spread to forward neighbors
        for s in range(len(b_pred)):
            if b_pred[s] > 0.01:  # Only spread from states with meaningful probability
                coord = self.mdp.maze.state_to_coord(s)
                # Try to spread 1-2 steps forward
                for steps in [1, 2]:
                    nr, nc = coord[0] + dr*steps, coord[1] + dc*steps
                    if (nr, nc) in self.mdp.maze._idx_map:
                        forward_s = self.mdp.maze.coord_to_state((nr, nc))
                        # Spread decreases with distance
                        forward_spread[forward_s] += b_pred[s] * (0.3 / steps)
        
        # Normalize forward spread
        if forward_spread.sum() > 0:
            forward_spread = forward_spread / forward_spread.sum()
            # Mix predicted belief with forward spreading
            b_pred = (1 - spreading_factor) * b_pred + spreading_factor * forward_spread
        
        self.belief = b_pred / b_pred.sum()

    def relocalise(self, marker_matrix): 
        b =  np.multiply(marker_matrix, self.belief)
        if b.sum() == 0:
            b = marker_matrix.copy()
        print(f"Relocalised belief: {b}")
        b /= b.sum()
        self.belief = b

    def select_action(self):
        if self.belief.max() < self.entropy_thresh:
            mp = int(np.argmax(self.belief))
            ci = self.mdp.maze.state_to_coord(mp)
            cps = [self.mdp.maze.state_to_coord(c) for c in self.mdp.maze.checkpoint_idxs]
            dists = [abs(ci[0]-y)+abs(ci[1]-x) for y,x in cps]
            target = cps[int(np.argmin(dists))]
            dy, dx = target[0]-ci[0], target[1]-ci[1]
            if abs(dy)>abs(dx):
                action = 'down' if dy>0 else 'up'
            else:
                action = 'right' if dx>0 else 'left'
            return self.mdp.actions.index(action)
        exp_q = self.belief @ self.mdp.Q
        return int(np.argmax(exp_q))

    def get_believed_position(self):
        idx = int(np.argmax(self.belief))
        return self.mdp.maze.state_to_coord(idx)
    

