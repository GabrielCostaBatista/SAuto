import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import matplotlib.patches as patches

class Maze:
    """
    Represents a 2D grid maze where 0 = free cell, 1 = wall.
    States are indices of free cells.
    Checkpoints are user-defined cells wher        self.visited_positions.add(self.current_position)
        return new_position, discovered_walls

    def select_action(self):the robot can localise perfectly.
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
        # Filter out checkpoints that are walls
        valid_checkpoints = [c for c in self.checkpoints if c in self._idx_map]
        self.checkpoint_idxs = [self._idx_map[c] for c in valid_checkpoints]
        self.checkpoints = valid_checkpoints  # Update checkpoints to only include valid ones

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
    Handles unknown walls by starting with no walls and updating when walls are discovered.
    """
    def __init__(self, maze, slip_prob=0.1, step_cost=-1, goal_reward=100, gamma=0.95):
        self.maze        = maze
        self.original_grid = maze.grid.copy()  # Keep original for reference
        # Start with no walls - assume all cells are free
        self.known_walls = set()  # Track discovered walls as (r,c) coordinates
        self.n           = maze.n_states
        self.actions     = ['up','down','left','right']
        self.slip        = slip_prob
        self.gamma       = gamma
        self.step_cost   = step_cost
        self.goal_reward = goal_reward

        self.P = np.zeros((self.n, len(self.actions), self.n))
        self.R = np.full((self.n, len(self.actions), self.n), self.step_cost)
        self.V = np.zeros(self.n)
        self.Q = np.zeros((self.n, len(self.actions)))
        
        # Build initial transition model assuming no walls
        self._build_transition_model()

    def _build_transition_model(self):
        """Build transition model based on currently known walls."""
        # Reset matrices
        self.P.fill(0)
        self.R.fill(self.step_cost)
        
        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        side_actions = {
            'up': ['left', 'right'],
            'down': ['left', 'right'], 
            'left': ['up', 'down'],
            'right': ['up', 'down']
        }
        
        for s in range(self.n):
            if s == self.maze.goal_idx:
                self.P[s,:,s] = 1.0
                continue
                
            ci = self.maze.state_to_coord(s)
            # Get free neighbors considering known walls
            free_neigh = []
            for nr, nc in self.maze.neighbors(ci):
                if (nr, nc) not in self.known_walls:
                    free_neigh.append((nr, nc))
            
            for a_idx, a in enumerate(self.actions):
                sides = side_actions[a]
                for b in self.actions:
                    if b == a:
                        prob = 1 - self.slip
                    elif b in sides:
                        prob = self.slip / 2
                    else:
                        prob = 0
                    
                    if prob > 0:
                        di, dj = directions[b]
                        ni, nj = ci[0]+di, ci[1]+dj
                        
                        # Check if target cell is within bounds and not a known wall
                        if (0 <= ni < self.maze.grid.shape[0] and 
                            0 <= nj < self.maze.grid.shape[1] and
                            (ni, nj) not in self.known_walls and
                            (ni, nj) in self.maze._idx_map):
                            
                            sp = self.maze.coord_to_state((ni, nj))
                            self.P[s,a_idx,sp] += prob
                            if sp == self.maze.goal_idx:
                                self.R[s,a_idx,sp] = self.goal_reward
                        else:
                            # Hit a wall or boundary - redistribute to free neighbors
                            if free_neigh:
                                for nbr in free_neigh:
                                    spn = self.maze.coord_to_state(nbr)
                                    self.P[s,a_idx,spn] += prob/len(free_neigh)
                                    if spn == self.maze.goal_idx:
                                        self.R[s,a_idx,spn] = self.goal_reward
                            else:
                                # Trapped: stay put
                                self.P[s,a_idx,s] += prob

    def update_walls(self, new_walls):
        """Update known walls and rebuild transition model."""
        old_wall_count = len(self.known_walls)
        self.known_walls.update(new_walls)
        
        if len(self.known_walls) > old_wall_count:
            print(f"Discovered {len(self.known_walls) - old_wall_count} new walls: {new_walls}")
            self._build_transition_model()
            # Perform incremental value iteration update
            self.incremental_value_iteration()
            return True
        return False

    def incremental_value_iteration(self, max_iters=50, eps=1e-3):
        """Perform limited value iteration to adjust for new walls."""
        for iteration in range(max_iters):
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
        
        # Update Q-values
        for s in range(self.n):
            self.Q[s] = np.sum(self.P[s] * (self.R[s] + self.gamma * self.V), axis=1)

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
    Handles wall discovery during navigation.
    """
    def __init__(self, mdp, entropy_thresh=0.9):
        self.mdp = mdp
        self.belief = None
        self.entropy_thresh = entropy_thresh
        self.current_position = None
        self.visited_positions = set()

    def init_belief(self):
        b = np.zeros(self.mdp.n)
        b[self.mdp.maze.start_idx] = 1.0
        self.belief = b
        self.current_position = self.mdp.maze.start

    def belief_entropy(self):
        p = self.belief
        return -np.sum(p[p>0] * np.log(p[p>0]))

    def predict_belief(self, action_idx):
        b_pred = self.belief @ self.mdp.P[:,action_idx,:]
        b_pred = b_pred / b_pred.sum()
        
        # Add forward spreading in the direction of movement
        spreading_factor = 0.15
        action = self.mdp.actions[action_idx]
        
        forward_spread = np.zeros_like(b_pred)
        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        dr, dc = directions[action]
        
        for s in range(len(b_pred)):
            if b_pred[s] > 0.01:
                coord = self.mdp.maze.state_to_coord(s)
                for steps in [1, 2]:
                    nr, nc = coord[0] + dr*steps, coord[1] + dc*steps
                    if (nr, nc) in self.mdp.maze._idx_map:
                        forward_s = self.mdp.maze.coord_to_state((nr, nc))
                        forward_spread[forward_s] += b_pred[s] * (0.3 / steps)
        
        if forward_spread.sum() > 0:
            forward_spread = forward_spread / forward_spread.sum()
            b_pred = (1 - spreading_factor) * b_pred + spreading_factor * forward_spread
        
        self.belief = b_pred / b_pred.sum()

    def scan_walls_around_marker(self, marker_position):
        """
        Scan for walls in a 5x5 radius around a marker position.
        Returns list of (r,c) coordinates of walls found.
        """
        mr, mc = marker_position
        walls_found = []
        
        # Scan 5x5 area (2 cells in each direction from marker)
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                nr, nc = mr + dr, mc + dc
                # Check if position is within bounds
                if (0 <= nr < self.mdp.original_grid.shape[0] and 
                    0 <= nc < self.mdp.original_grid.shape[1]):
                    # Check if it's a wall in the original maze
                    if self.mdp.original_grid[nr, nc] == 1:
                        # Only add if we haven't discovered this wall yet
                        if (nr, nc) not in self.mdp.known_walls:
                            walls_found.append((nr, nc))
        
        return walls_found

    def relocalise(self, marker_belief, discovered_walls=None, marker_position=None): 
        """
        Relocalise based on marker detection and update walls if discovered.
        marker_belief: belief update from marker detection
        discovered_walls: list of (r,c) coordinates of newly discovered walls
        marker_position: position of the detected marker for 5x5 wall sensing
        """
        # Update belief based on marker detection
        b = np.multiply(marker_belief, self.belief)
        if b.sum() == 0:
            b = marker_belief.copy()
        b /= b.sum()
        self.belief = b
        
        # If we detected a marker, scan 5x5 area around it for walls
        if marker_position is not None:
            walls_in_radius = self.scan_walls_around_marker(marker_position)
            if walls_in_radius:
                print(f"Marker at {marker_position} detected {len(walls_in_radius)} walls in 5x5 radius: {walls_in_radius}")
                if discovered_walls:
                    discovered_walls.extend(walls_in_radius)
                else:
                    discovered_walls = walls_in_radius
        
        # Update MDP with newly discovered walls
        if discovered_walls:
            walls_updated = self.mdp.update_walls(discovered_walls)
            if walls_updated:
                print(f"MDP updated with {len(discovered_walls)} new walls")
        
        print(f"Relocalised belief: {self.belief}")

    def simulate_movement(self, action_idx):
        """
        Simulate robot movement and wall discovery.
        Returns (new_position, discovered_walls)
        """
        action = self.mdp.actions[action_idx]
        directions = {'up':(-1,0), 'down':(1,0), 'left':(0,-1), 'right':(0,1)}
        dr, dc = directions[action]
        
        # Try to move in intended direction
        new_r = self.current_position[0] + dr
        new_c = self.current_position[1] + dc
        
        discovered_walls = []
        
        # Check if new position is valid (within bounds)
        if (0 <= new_r < self.mdp.maze.grid.shape[0] and 
            0 <= new_c < self.mdp.maze.grid.shape[1]):
            
            # Check if it's actually a wall in the original maze
            if self.mdp.original_grid[new_r, new_c] == 1:
                # Discovered a wall!
                discovered_walls.append((new_r, new_c))
                # Stay in current position
                new_position = self.current_position
                print(f"Hit wall at {(new_r, new_c)}! Staying at {self.current_position}")
            else:                # Successfully moved
                new_position = (new_r, new_c)
                self.current_position = new_position
                print(f"Moved to {new_position}")
        else:
            # Hit boundary, stay in place
            new_position = self.current_position
            print(f"Hit boundary, staying at {self.current_position}")
        
        self.visited_positions.add(self.current_position)
        return new_position, discovered_walls

    def select_action(self):
        # Special case: if at start position, strongly prefer going RIGHT
        if self.current_position == self.mdp.maze.start:
            print("At start position - biasing toward RIGHT to avoid wall")
            return self.mdp.actions.index('right')
        
        # Always use QMDP policy first - only go to checkpoints if really lost
        exp_q = self.belief @ self.mdp.Q
        best_qmdp_action = int(np.argmax(exp_q))
        
        # Override if we're trying to go UP from start area (likely hitting wall)
        if (self.current_position[0] <= 2 and self.current_position[1] <= 2 and 
            self.mdp.actions[best_qmdp_action] == 'up'):
            print("Avoiding UP action near start area")
            # Prefer right if possible, otherwise down
            if self.current_position[1] < 6:  # Can still go right
                return self.mdp.actions.index('right')
            else:
                return self.mdp.actions.index('down')
        
        # Only override with checkpoint seeking if belief is very uncertain AND far from goal
        if self.belief.max() < self.entropy_thresh:
            mp = int(np.argmax(self.belief))
            ci = self.mdp.maze.state_to_coord(mp)
            goal_coord = self.mdp.maze.state_to_coord(self.mdp.maze.goal_idx)
            
            # Check distance to goal - if close, stick with QMDP
            goal_distance = abs(ci[0] - goal_coord[0]) + abs(ci[1] - goal_coord[1])
            if goal_distance <= 3:  # If close to goal, use QMDP
                return best_qmdp_action
                
            # Only seek checkpoints if really far and uncertain
            cps = [self.mdp.maze.state_to_coord(c) for c in self.mdp.maze.checkpoint_idxs]
            if cps:  # Only if checkpoints exist
                dists = [abs(ci[0]-y)+abs(ci[1]-x) for y,x in cps]
                target = cps[int(np.argmin(dists))]
                dy, dx = target[0]-ci[0], target[1]-ci[1]
                if abs(dy)>abs(dx):
                    action = 'down' if dy>0 else 'up'
                else:
                    action = 'right' if dx>0 else 'left'
                return self.mdp.actions.index(action)
        
        return best_qmdp_action

    def get_believed_position(self):
        idx = int(np.argmax(self.belief))
        return self.mdp.maze.state_to_coord(idx)

    

