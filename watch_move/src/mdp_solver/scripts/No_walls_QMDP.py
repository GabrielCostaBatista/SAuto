import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import matplotlib.patches as patches
import matplotlib.animation as animation

class Maze:
    """
    Represents a 2D grid maze where 0 = free cell, 1 = wall.  
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
    def __init__(self, original_grid, start, goal, checkpoints=None, slip_prob=0.1, step_cost=-1, goal_reward=100, gamma=0.95):
        # Keep original grid for wall discovery reference
        self.original_grid = np.array(original_grid)
        
        # Create a completely open maze initially (no walls except boundaries)
        self.open_grid = np.zeros_like(self.original_grid)
        # Set boundaries as walls (first/last row and column)
        self.open_grid[0, :] = 1   # Top boundary
        self.open_grid[-1, :] = 1  # Bottom boundary  
        self.open_grid[:, 0] = 1   # Left boundary
        self.open_grid[:, -1] = 1  # Right boundary
        
        # Create maze with the open grid (no internal walls initially)
        self.maze = Maze(self.open_grid, start, goal, checkpoints)
        
        # Track discovered walls as (r,c) coordinates
        self.known_walls = set()
        
        self.n           = self.maze.n_states
        self.actions     = ['up','down','left','right']
        self.slip        = slip_prob
        self.gamma       = gamma
        self.step_cost   = step_cost
        self.goal_reward = goal_reward

        self.P = np.zeros((self.n, len(self.actions), self.n))
        self.R = np.full((self.n, len(self.actions), self.n), self.step_cost)
        self.V = np.zeros(self.n)
        self.Q = np.zeros((self.n, len(self.actions)))
        
        # Track Value Function evolution over time
        self.V_history = []
        self.wall_discovery_steps = []  # Track when walls were discovered
        
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
            # Perform small value iteration update
            self.incremental_value_iteration()
            # Save Value Function snapshot after wall discovery
            self.V_history.append(self.V.copy())
            self.wall_discovery_steps.append(len(self.V_history) - 1)
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
        
        # Save initial Value Function
        self.V_history.append(self.V.copy())



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
        # Special case
        if self.current_position == self.mdp.maze.start:
            print("At start position - biasing toward RIGHT to avoid wall")
            return self.mdp.actions.index('right')
        
        # Always use QMDP policy first - only go to checkpoints if really lost
        exp_q = self.belief @ self.mdp.Q
        best_qmdp_action = int(np.argmax(exp_q))
        
        
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


class MazeVisualizer:
    """
    Visualizes the maze, robot path, belief state, and wall discovery process.
    """
    def __init__(self, maze, controller):
        self.maze = maze
        self.controller = controller
        self.path_history = []
        self.belief_history = []
        self.discovered_walls_history = []
        self.position_history = []  # Add this to track positions
        self.fig = None
        self.ax = None  # Single axis instead of multiple axes
        self.belief_colorbar = None  # Store colorbar reference

    def setup_plots(self):
        """Setup the visualization plots."""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        
        
        # Define colors for maze visualization
        self.colors = ['white', 'black', 'lightgray', 'red', 'green', 'blue', 'orange']
        self.cmap = ListedColormap(self.colors)

    def update_history(self, position, belief, discovered_walls):
        """Update the history for visualization."""
        self.path_history.append(position)
        self.position_history.append(position)  # Track positions separately
        self.belief_history.append(belief.copy())
        self.discovered_walls_history.append(discovered_walls.copy() if discovered_walls else set())

    def plot_maze_and_path(self, ax, step):
        """Plot the maze with current path and discovered walls."""
        ax.clear()
        
        # Create maze visualization grid
        maze_vis = self.maze.grid.copy().astype(float)
        
        # Mark discovered walls differently
        all_discovered = set()
        for walls in self.discovered_walls_history:
            all_discovered.update(walls)
        
        for wall in all_discovered:
            if wall[0] < maze_vis.shape[0] and wall[1] < maze_vis.shape[1]:
                maze_vis[wall] = 2  # Light gray for discovered walls
        
        # Mark start and goal
        maze_vis[self.maze.start] = 3  # Red for start
        maze_vis[self.maze.goal] = 4   # Green for goal
        
        # Mark checkpoints
        for checkpoint in self.maze.checkpoints:
            maze_vis[checkpoint] = 5  # Blue for checkpoints
            
        # Mark current position
        if self.path_history:
            current_pos = self.path_history[-1]
            maze_vis[current_pos] = 6  # Orange for current position
          
        ax.imshow(maze_vis, cmap=self.cmap, vmin=0, vmax=6, origin='upper')
        
        # Plot path
        if len(self.path_history) > 1:
            path_y, path_x = zip(*self.path_history)
            ax.plot(path_x, path_y, 'purple', linewidth=3, alpha=0.7, label='Robot Path')
        
        ax.set_xlabel('x coordinate (columns)', fontsize=16)
        ax.set_ylabel('y coordinate (rows)', fontsize=16)
        ax.grid(True, alpha=0.3)
          # Add legend
        legend_elements = [
            patches.Patch(color='white', label='Free Space'),
            patches.Patch(color='black', label='Unknown Walls'),
            patches.Patch(color='lightgray', label='Discovered Walls'),
            patches.Patch(color='red', label='Start'),
            patches.Patch(color='green', label='Goal'),
            patches.Patch(color='blue', label='Checkpoints'),
            patches.Patch(color='orange', label='Current Position')
        ]        
        
        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize=16)
        
    def plot_belief_state(self, ax, step):
        """Plot the current belief state as a heatmap."""
        ax.clear()
        
        # Create belief grid
        belief_grid = np.zeros(self.maze.grid.shape)
        if self.belief_history:
            current_belief = self.belief_history[-1]
            for state_idx, prob in enumerate(current_belief):
                if prob > 0:
                    coord = self.maze.state_to_coord(state_idx)
                    belief_grid[coord] = prob
        im = ax.imshow(belief_grid, cmap='Reds', origin='upper', vmin=0, vmax=1)
        ax.set_title(f'Belief State - Step {step}')
        ax.set_xlabel('X coordinate')
        ax.set_ylabel('Y coordinate')
        
        # Add colorbar only if it doesn't exist
        if self.belief_colorbar is None:
            self.belief_colorbar = plt.colorbar(im, ax=ax)
            self.belief_colorbar.set_label('Belief Probability')
        
        # Mark walls
        wall_y, wall_x = np.where(self.maze.grid == 1)
        ax.scatter(wall_x, wall_y, c='black', marker='s', s=50, alpha=0.5)
        
    def plot_discovery_progress(self, ax):
        """Plot wall discovery progress over time."""
        ax.clear()
        
        steps = list(range(len(self.discovered_walls_history)))
        cumulative_walls = []
        total_discovered = set()
        
        for walls in self.discovered_walls_history:
            total_discovered.update(walls)
            cumulative_walls.append(len(total_discovered))
        
        total_walls = len(np.where(self.maze.grid == 1)[0])
        
        ax.plot(steps, cumulative_walls, 'b-', linewidth=2, label='Discovered Walls')
        ax.axhline(y=total_walls, color='r', linestyle='--', label=f'Total Walls ({total_walls})')
        
        ax.set_title('Wall Discovery Progress')
        ax.set_xlabel('Step')
        ax.set_ylabel('Number of Walls Discovered')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def plot_belief_entropy(self, ax):
        """Plot belief entropy over time."""
        ax.clear()
        
        entropies = []
        for belief in self.belief_history:
            p = belief[belief > 0]
            entropy = -np.sum(p * np.log(p)) if len(p) > 0 else 0
            entropies.append(entropy)
        
        steps = list(range(len(entropies)))
        ax.plot(steps, entropies, 'g-', linewidth=2)
        ax.set_title('Belief Entropy Over Time')
        ax.set_xlabel('Step')
        ax.set_ylabel('Entropy (nats)')
        ax.grid(True, alpha=0.3)

    def update_visualization(self, step):
        """Update visualization to show only the maze and path with discovered walls."""
        if self.fig is None:
            self.setup_plots()
            
        # Only plot the maze and path - no animation, just final state
        self.plot_maze_and_path(self.ax, step)
        
        plt.tight_layout()
        plt.pause(0.01)  # Very brief pause for display update

    def save_final_plot(self, filename='maze_navigation_result.png'):
        """Save the final visualization."""
        if self.fig is not None:
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Visualization saved as {filename}")    

    def create_final_value_function_plot(self, mdp):
        """Create a static plot showing the final Value Function after all wall discoveries."""
        if not mdp.V_history:
            print("No Value Function history available for plot.")
            return
            
        print(f"Creating final Value Function plot after {len(mdp.V_history)} updates...")
        
        # Setup figure
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # Get final Value Function
        final_V = mdp.V_history[-1]
          # Create Value Function grid
        value_grid = np.full(mdp.maze.grid.shape, np.nan)
        for state_idx, value in enumerate(final_V):
            coord = mdp.maze.state_to_coord(state_idx)
            value_grid[coord] = value
        
        # Mark discovered walls as black squares on the value grid
        # Use a very low value (much lower than any actual value) to make them appear black
        min_value = np.nanmin(value_grid)
        wall_value = min_value - abs(min_value) * 0.5  # Make it notably darker than the lowest value
        
        for wall_coord in mdp.known_walls:
            if (0 <= wall_coord[0] < value_grid.shape[0] and 
                0 <= wall_coord[1] < value_grid.shape[1]):
                value_grid[wall_coord] = wall_value
          # Plot Value Function heatmap
        im = ax.imshow(value_grid, cmap='viridis', origin='upper', interpolation='nearest')
        
        ax.set_xlabel('x coordinate (columns)', fontsize=16)
        ax.set_ylabel('y coordinate (rows)', fontsize=16)

        # Add colorbar with larger label
        cbar = plt.colorbar(im, ax=ax, label='Value')
        cbar.set_label('Value', fontsize=16)
        cbar.ax.tick_params(labelsize=12)
          # Mark boundary walls (these are always known) with red outline for distinction
        boundary_walls_y, boundary_walls_x = np.where(mdp.maze.grid == 1)
        ax.scatter(boundary_walls_x, boundary_walls_y, c='none', marker='s', s=30, 
                  edgecolors='red', linewidth=1, alpha=0.6, label='Boundary Walls')
        
        start_y, start_x = mdp.maze.start
        goal_y, goal_x = mdp.maze.goal
        ax.scatter(start_x, start_y, c='white', marker='o', s=100, 
                   edgecolors='black', linewidth=2, label='Start')
        ax.scatter(goal_x, goal_y, c='yellow', marker='*', s=200, 
                   edgecolors='black', linewidth=2, label='Goal')
        
        # Increase axis tick label size
        ax.tick_params(axis='both', which='major', labelsize=12)
        
        # Add a text note about discovered walls
        ax.text(0.02, 0.98, f'Black squares: {len(mdp.known_walls)} discovered walls', 
                transform=ax.transAxes, fontsize=12, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        ax.legend(loc='upper right', fontsize=12)
        
        plt.tight_layout()
        plt.show()
        
        return fig


# Simulation code
if __name__ == "__main__":
    maze_grid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]
    start, goal = (1,1), (9,11)
    checkpoints = [(1,6), (9,6), (9,11)]
    print("Creating maze and MDP...")
    # Create MDP with original grid - it will create its own open maze internally
    mdp = MDP(maze_grid, start, goal, checkpoints, slip_prob=0.1)
    print("Performing initial value iteration (assuming no walls)...")
    mdp.value_iteration()
    
    print("Creating controller...")
    controller = QMDPController(mdp, entropy_thresh=0.3)
    controller.init_belief()    # Immediately detect start marker for perfect initial localization
    print("Detecting start marker for initial localization...")
    marker_belief = np.zeros(mdp.n)
    marker_belief[mdp.maze.coord_to_state(start)] = 1.0
    # Pass the start position to enable 5x5 wall scanning around start
    controller.relocalise(marker_belief, marker_position=start)
    
    # Create visualizer
    print("Setting up visualization...")
    visualizer = MazeVisualizer(mdp.maze, controller)
    
    print(f"Starting simulation from {start} to {goal}")
    print(f"Original maze:\n{maze_grid}")
    print(f"Known walls at start: {mdp.known_walls}")
    plt.ion()
    
    max_steps = 200 
    step = 0
    
    while step < max_steps:
        current_pos = controller.current_position
        print(f"\n--- Step {step + 1} ---")
        print(f"Current position: {current_pos}")
        print(f"Current belief peak at: {controller.get_believed_position()}")
        visualizer.update_history(current_pos, controller.belief, mdp.known_walls)
        
        # Check if reached goal
        if current_pos == goal:
            print(f"SUCCESS! Reached goal {goal} in {step} steps!")
            break
        
        # Select action using QMDP policy
        action_idx = controller.select_action()
        action = mdp.actions[action_idx]
        print(f"Selected action: {action}")
        
        # Simulate movement and wall discovery
        new_pos, discovered_walls = controller.simulate_movement(action_idx)
        
        # Update belief based on movement
        controller.predict_belief(action_idx)
          # If walls were discovered, update MDP and relocalise
        if discovered_walls:
            # Create marker belief (in real scenario, this would come from sensors)
            marker_belief = np.zeros(mdp.n)
            # Assume we know our exact position when hitting a wall
            if new_pos in mdp.maze._idx_map:
                marker_belief[mdp.maze.coord_to_state(new_pos)] = 1.0
            else:
                marker_belief = np.ones(mdp.n) / mdp.n  # Uniform if unsure
            
            controller.relocalise(marker_belief, discovered_walls)
        if step % 5 == 0 and checkpoints:  
            for checkpoint in checkpoints:
                if (abs(current_pos[0] - checkpoint[0]) + 
                    abs(current_pos[1] - checkpoint[1])) <= 2:  # Increased detection range                    print(f"Detected marker near {checkpoint}!")
                    marker_belief = np.zeros(mdp.n)
                    # Assume marker gives us precise location
                    marker_belief[mdp.maze.coord_to_state(current_pos)] = 1.0
                    # Pass the checkpoint position to enable 5x5 wall scanning
                    controller.relocalise(marker_belief, marker_position=checkpoint)
                    break
        
        step += 1
        
        # Safety check
        if len(mdp.known_walls) >= len(np.where(maze_grid == 1)[0]):
            print("All walls discovered!")
    
    if step >= max_steps:
        print(f"Simulation ended after {max_steps} steps without reaching goal.")
    
    # Final visualization update
    visualizer.update_history(controller.current_position, controller.belief, mdp.known_walls)
    visualizer.update_visualization(step + 1)
    # Save the final result
    visualizer.save_final_plot('qmdp_navigation_result.png')
    
    print(f"\nFinal statistics:")
    print(f"Total walls discovered: {len(mdp.known_walls)}")
    print(f"Discovered walls: {mdp.known_walls}")
    print(f"Actual walls in maze: {set(zip(*np.where(maze_grid == 1)))}")
    print(f"Final position: {controller.current_position}")
    print(f"Goal position: {goal}")
    print("\nCreating final Value Function plot...")
    try:
        fig = visualizer.create_final_value_function_plot(mdp)
        print("Final Value Function plot created successfully!")
    except Exception as e:
        print(f"Could not create Value Function plot: {e}")
    
    # Keep the plot open
    plt.ioff()
    plt.show()