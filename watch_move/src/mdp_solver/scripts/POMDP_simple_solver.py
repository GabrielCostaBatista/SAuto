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
        self.n_states = idx
        self.start_idx = self._idx_map[self.start]
        self.goal_idx = self._idx_map[self.goal]
        self.checkpoint_idxs = [self._idx_map[c] for c in self.checkpoints]

    def state_to_coord(self, s):
        return self._rev_map[s]

    def coord_to_state(self, coord):
        return self._idx_map[tuple(coord)]
    
    def coord_to_state(self, coord):
        return self._idx_map[tuple(coord)]

    def neighbors(self, coord):
        """
        Yields adjacent free-cell coordinates (up, down, left, right) from a given (row, col).
        """
        i, j = coord
        for di, dj in [(-1,0), (1,0), (0,-1), (0,1)]:
            ni, nj = i + di, j + dj
            if 0 <= ni < self.grid.shape[0] and 0 <= nj < self.grid.shape[1] and self.grid[ni, nj] == 0:
                yield (ni, nj)
        
    def __repr__(self):
        return f"Maze(start={self.start}, goal={self.goal}, checkpoints={self.checkpoints})"

class MDP:
    """
    Computes optimal Q and V using value iteration for the goal-reaching task.
    Modified so that bumping into a wall redistributes motion to valid neighbors.
    """
    def __init__(self, maze, slip_prob=0.1, step_cost=-1, goal_reward=100, gamma=0.95):
        self.maze = maze
        self.n = maze.n_states
        self.actions = ['up','down','left','right']
        self.slip = slip_prob
        self.gamma = gamma
        self.step_cost = step_cost
        self.goal_reward = goal_reward
        # Transition and Reward
        self.P = np.zeros((self.n, len(self.actions), self.n))
        self.R = np.full((self.n, len(self.actions), self.n), self.step_cost)

        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        for s in range(self.n):
            if s == maze.goal_idx:
                self.P[s,:,s] = 1.0
                continue
            ci = maze.state_to_coord(s)
            free_neigh = list(maze.neighbors(ci))
            for a_idx, a in enumerate(self.actions):
                for b in self.actions:
                    prob = (1 - self.slip) if b == a else self.slip/3
                    di, dj = directions[b]
                    ni, nj = ci[0]+di, ci[1]+dj
                    if (ni, nj) in maze._idx_map:
                        sp = maze.coord_to_state((ni, nj))
                        self.P[s,a_idx,sp] += prob
                        if sp == maze.goal_idx:
                            self.R[s,a_idx,sp] = self.goal_reward
                    else:
                        # redistributive bump: spread to free neighbors
                        if free_neigh:
                            for nbr in free_neigh:
                                spn = maze.coord_to_state(nbr)
                                self.P[s,a_idx,spn] += prob / len(free_neigh)
                                if spn == maze.goal_idx:
                                    self.R[s,a_idx,spn] = self.goal_reward
                        else:
                            # trapped: stay
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
                max_q = np.max(q_vals)
                delta = max(delta, abs(max_q - self.V[s]))
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
        self.belief = b_pred / b_pred.sum()

    def relocalise(self, state_idx):
        b = np.zeros(self.mdp.n)
        b[state_idx] = 1.0
        self.belief = b

    def select_action(self):
        if self.belief.max() < self.entropy_thresh:
            mp = int(np.argmax(self.belief))
            ci = self.mdp.maze.state_to_coord(mp)
            cps = [self.mdp.maze.state_to_coord(c) for c in self.mdp.maze.checkpoint_idxs]
            dists = [abs(ci[0]-y)+abs(ci[1]-x) for y,x in cps]
            target = cps[int(np.argmin(dists))]
            dy, dx = target[0]-ci[0], target[1]-ci[1]
            if abs(dy)>abs(dx): action = 'down' if dy>0 else 'up'
            else: action = 'right' if dx>0 else 'left'
            return self.mdp.actions.index(action)
        exp_q = self.belief @ self.mdp.Q
        return int(np.argmax(exp_q))

    def control_loop(self, send_action, check_goal, detect_checkpoint, max_steps=200):
        self.init_belief()
        state = self.mdp.maze.start_idx
        true_path = [self.mdp.maze.state_to_coord(state)]
        believed_path = [self.mdp.maze.state_to_coord(int(np.argmax(self.belief)))]
        for _ in range(max_steps):
            a = self.select_action()
            coord = send_action(a)
            s_idx = self.mdp.maze.coord_to_state(coord)
            if detect_checkpoint(coord): self.relocalise(s_idx)
            else: self.predict_belief(a)
            true_path.append(coord)
            b_coord = self.mdp.maze.state_to_coord(int(np.argmax(self.belief)))
            believed_path.append(b_coord)
            if check_goal(coord): break
        return true_path, believed_path
    
    def get_believed_position(self):
        """
        Returns the (row, col) coordinate the robot currently believes it is in.
        """
        idx = int(np.argmax(self.belief))
        return self.mdp.maze.state_to_coord(idx)
