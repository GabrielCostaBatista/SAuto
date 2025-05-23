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
        # checkpoint state indices
        self.checkpoint_idxs = [self._idx_map[c] for c in self.checkpoints]

    def state_to_coord(self, s):
        return self._rev_map[s]

    def coord_to_state(self, coord):
        return self._idx_map[tuple(coord)]

class MDP:
    """
    Computes optimal Q and V using value iteration for the goal-reaching task.
    """
    def __init__(self, maze, slip_prob=0.1, step_cost=-1, goal_reward=100, gamma=0.95):
        self.maze = maze
        self.n = maze.n_states
        self.actions = ['up','down','left','right']
        self.slip = slip_prob
        self.gamma = gamma
        self.step_cost = step_cost
        self.goal_reward = goal_reward
        # Transition (P) and reward (R) tensors
        self.P = np.zeros((self.n, len(self.actions), self.n))
        self.R = np.full((self.n, len(self.actions), self.n), self.step_cost)

        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        for s in range(self.n):
            if s == maze.goal_idx:
                self.P[s,:,s] = 1.0
                continue
            ci = maze.state_to_coord(s)
            for a_idx, a in enumerate(self.actions):
                for b in self.actions:
                    prob = (1 - self.slip) if b == a else self.slip / 3
                    ni, nj = ci[0] + directions[b][0], ci[1] + directions[b][1]
                    sp = s
                    if (ni, nj) in maze._idx_map:
                        sp = maze.coord_to_state((ni, nj))
                    self.P[s, a_idx, sp] += prob
                    if sp == maze.goal_idx:
                        self.R[s, a_idx, sp] = self.goal_reward
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
        # Initialise to known start position
        b = np.zeros(self.mdp.n)
        b[self.mdp.maze.start_idx] = 1.0
        self.belief = b

    def belief_entropy(self):
        p = self.belief
        # avoid log(0)
        return -np.sum(p[p>0] * np.log(p[p>0]))

    def predict_belief(self, action_idx):
        # Propagate belief using transition model
        b_pred = self.belief @ self.mdp.P[:, action_idx, :]
        self.belief = b_pred / b_pred.sum()

    def relocalise(self, state_idx):
        # On checkpoint detection, collapse belief
        b = np.zeros(self.mdp.n)
        b[state_idx] = 1.0
        self.belief = b

    def select_action(self):
        # If too uncertain, seek nearest checkpoint
        if self.belief.max() < self.entropy_thresh:
            # most probable coord
            mp = int(np.argmax(self.belief))
            ci = self.mdp.maze.state_to_coord(mp)
            # find nearest checkpoint
            cps = [self.mdp.maze.state_to_coord(c) for c in self.mdp.maze.checkpoint_idxs]
            dists = [abs(ci[0]-y)+abs(ci[1]-x) for y,x in cps]
            target = cps[int(np.argmin(dists))]
            dy = target[0] - ci[0]; dx = target[1] - ci[1]
            if abs(dy) > abs(dx):
                action = 'down' if dy > 0 else 'up'
            else:
                action = 'right' if dx > 0 else 'left'
            return self.mdp.actions.index(action)
        # else exploit QMDP towards goal
        exp_q = self.belief @ self.mdp.Q
        return int(np.argmax(exp_q))

    def control_loop(self, send_action, check_goal, detect_checkpoint, max_steps=200):
        """
        send_action(a_idx)->state_idx
        check_goal(state_idx)->bool
        detect_checkpoint(state_idx)->bool
        """
        self.init_belief()
        true_state = self.mdp.maze.start_idx
        true_path, believed_path = [self.mdp.maze.state_to_coord(true_state)], [self.mdp.maze.state_to_coord(np.argmax(self.belief))]

        for _ in range(max_steps):
            a_idx = self.select_action()
            # execute action
            true_state = send_action(a_idx)
            true_path.append(self.mdp.maze.state_to_coord(true_state))
            # check for checkpoint
            if detect_checkpoint(true_state):
                self.relocalise(true_state)
            else:
                self.predict_belief(a_idx)
            believed_path.append(self.mdp.maze.state_to_coord(np.argmax(self.belief)))
            # goal check
            if check_goal(true_state):
                break
        return true_path, believed_path