import numpy as np

env_grid = np.array([
   [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
   [0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
   [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
   [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
   [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
   [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1],
   [0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
   [0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1],
   [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
   [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0],
   [0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1],
   [0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
   [0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
   [1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1],
   [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0],
   [0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1],
   [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1],
   [0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1]])
n_rows, n_cols = env_grid.shape

start = (0, 0)                # In this case, the robot starts at the top left corner
goal  = (0, n_cols - 1)         # In this case, the robot starts at the top right corner


checkpoints = [(0, 3), (2, 5), (4, 7)]  # Example checkpoints

class Maze:
    def __init__(self, grid, start, goal, checkpoints=None):
        self.grid = np.array(grid)
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.checkpoints = [tuple(c) for c in (checkpoints or [])]
        self._idx_map, self._rev_map = {}, {}
        
        idx = 0
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i,j] == 0:
                    self._idx_map[(i,j)] = idx
                    self._rev_map[idx] = (i,j)
                    idx += 1
        self.n_states  = idx

        self.start_idx = self._idx_map[self.start]
        self.goal_idx  = self._idx_map[self.goal]
        self.markers   = [self._idx_map[c] for c in self.checkpoints]

    def state_to_coord(self, s):
        return self._rev_map[s]

    def step(self, state, action):
        di_dj = {'up':(-1,0),'down':(1,0),'left':(0,-1),'right':(0,1)}
        x,y = self._rev_map[state]
        dx,dy = di_dj[action]
        nx,ny = x+dx, y+dy
        if 0<=nx<self.grid.shape[0] and 0<=ny<self.grid.shape[1] and self.grid[nx,ny]==0:
            return self._idx_map[(nx,ny)]
        return state

class BeliefGreedySearch:
    def __init__(self, maze, slip_prob=0.2, marker_reward=10):
        self.maze = maze
        self.start = maze.start_idx
        self.goal  = maze.goal_idx
        self.states = list(range(maze.n_states))
        uniform_p = 1.0 / maze.n_states
        self.belief = {s: uniform_p for s in self.states}
        self.slip_prob = slip_prob
        self.marker_reward = marker_reward
        # internal map of walls: -1 unknown, 0 free, 1 wall
        self.known_grid = -np.ones_like(self.maze.grid)
        # start cell is known free
        x0,y0 = self.maze.state_to_coord(self.start)
        self.known_grid[x0,y0] = 0

    def sense_and_update(self, coord):
        """Sense adjacent walls and update known map."""
        x,y = coord
        for a, (dx,dy) in {'up':(-1,0),'down':(1,0),'left':(0,-1),'right':(0,1)}.items():
            nx,ny = x+dx, y+dy
            if 0<=nx<self.maze.grid.shape[0] and 0<=ny<self.maze.grid.shape[1]:
                # update known grid with actual from full grid
                self.known_grid[nx,ny] = 1 if self.maze.grid[nx,ny]==1 else 0

    def bfs_plan(self, start, goal):
        """Return list of actions from start->goal on known_grid using BFS."""
        from collections import deque
        visited={start:None}
        queue=deque([start])
        while queue:
            cur=queue.popleft()
            if cur==goal: break
            x,y=cur
            for a,(dx,dy) in {'up':(-1,0),'down':(1,0),'left':(0,-1),'right':(0,1)}.items():
                nx,ny=x+dx,y+dy
                # traverse free or unknown cells (not known walls)
                if 0<=nx<self.known_grid.shape[0] and 0<=ny<self.known_grid.shape[1] and self.known_grid[nx,ny]!=1:
                     if (nx,ny) not in visited:
                         visited[(nx,ny)]=(cur,a)
                         queue.append((nx,ny))
         # reconstruct
        if goal not in visited:
            return []  # no path found
        actions=[]; cur=goal
        while visited[cur] is not None:
             prev,a=visited[cur]
             actions.append(a); cur=prev
        return actions[::-1]

    def manhattan(self, s):
        x1,y1 = self.maze.state_to_coord(s)
        x2,y2 = self.maze.state_to_coord(self.goal)
        return abs(x1-x2) + abs(y1-y2)

    def expected_heuristic(self, belief):
        return sum(p * self.manhattan(s) for s,p in belief.items())

    def reward(self, state):
        r = -1
        if state in self.maze.markers:
            r += self.marker_reward
        return r

    def transition(self, s, a):
        intended = self.maze.step(s, a)
        right = {'up':'right','right':'down','down':'left','left':'up'}[a]
        slipped = self.maze.step(s, right)
        return [(intended, 1-self.slip_prob), (slipped, self.slip_prob)]

    def update_belief(self, belief, a):
        new_b = {s:0.0 for s in belief}
        for s,p in belief.items():
            for s2,pt in self.transition(s,a):
                new_b[s2] += p * pt
        tot = sum(new_b.values()) or 1.0
        return {s: p/tot for s,p in new_b.items()}

    def observe(self, belief, a):
        b_pred = self.update_belief(belief, a)
        saw = any(p>0.5 and s in self.maze.markers for s,p in b_pred.items())
        new_b = {s: ((1.0 if (s in self.maze.markers and saw) else 0.1) * p)
                 for s,p in b_pred.items()}
        tot = sum(new_b.values()) or 1.0
        return {s: p/tot for s,p in new_b.items()}

    def search(self):
        """Reactive greedy: move, sense walls, then choose the neighbour
           with the smallest Manhattan distance to the goal."""
        real = self.start
        visited = {real}            # never go back to a cell

        path = []
        # precompute goal coords
        goal_x, goal_y = self.maze.state_to_coord(self.goal)
        i = 0

        # directions
        actions = {'up':(-1,0),'down':(1,0),'left':(0,-1),'right':(0,1)}

        while real != self.goal:
            x, y = self.maze.state_to_coord(real)
            # sense walls
            self.sense_and_update((x, y))

            # pick best next action
            best_a, best_dist = None, float('inf')
            for a, (dx, dy) in actions.items():
                # simulate move to get the candidate state
                s2 = self.maze.step(real, a)
                if s2 == real: 
                    continue          # hit a wall or out‐of‐bounds
                if s2 in visited:
                    continue          # don’t revisit
                nx, ny = self.maze.state_to_coord(s2)
                # skip known walls too, just in case
                if not (0 <= nx < self.known_grid.shape[0] and 
                        0 <= ny < self.known_grid.shape[1]):
                    continue
                if self.known_grid[nx, ny] == 1:
                    continue
                # compute manhattan to goal
                dist = abs(nx - goal_x) + abs(ny - goal_y)
                if dist < best_dist:
                    best_dist = dist
                    best_a    = a

            if best_a is None:
                # ←– Fallback to BFS on the known grid
                bfs_actions = self.bfs_plan((x, y), (goal_x, goal_y))
                if not bfs_actions:
                    raise RuntimeError(f"No path found from {(x,y)} to goal")
                best_a = bfs_actions[0]    # take the first step of the BFS plan

            # execute move
            real = self.maze.step(real, best_a)
            visited.add(real)        # mark the new cell as seen
            path.append(best_a)

        return path


        #raise RuntimeError("Search exceeded iteration limit")

# Instantiate and run
maze = Maze(env_grid, start, goal, checkpoints)
solver = BeliefGreedySearch(maze)
actions = solver.search()
# Reconstruct path
coords = []
s = maze.start_idx
coords.append(maze.state_to_coord(s))
for a in actions:
    s = maze.step(s,a)
    coords.append(maze.state_to_coord(s))
