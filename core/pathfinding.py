"""
Phase 5 (patched): A* pathfinding + RWARE action conversion with
decentralized collision avoidance.

Fixes over the previous version:
  1. Robots carrying a shelf treat other shelf cells as obstacles
     (RWARE refuses to move a loaded robot onto a shelf cell).
  2. After a delivery the robot RETURNS the shelf to its home cell and
     drops it there (TOGGLE_LOAD). Previously robots stayed loaded forever.
  3. A robot that is carrying a shelf always acts on it, regardless of
     the auction's pursue flag (otherwise it froze mid-delivery).
  4. Other robots' cells are treated as obstacles via a replan; if no
     route exists the robot waits. A stuck-breaker sidesteps after
     STUCK_LIMIT steps without progress.
  5. Same-cell conflicts: lower robot_id has priority (unchanged).
"""

import heapq


class GridPathfinder:
    NOOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    TOGGLE_LOAD = 4

    UP, DOWN, LEFT_DIR, RIGHT_DIR = 0, 1, 2, 3
    DELTA = {UP: (0, -1), DOWN: (0, 1), LEFT_DIR: (-1, 0), RIGHT_DIR: (1, 0)}

    # clockwise ordering used to compute shortest turn direction
    CLOCKWISE_ORDER = [UP, RIGHT_DIR, DOWN, LEFT_DIR]

    STUCK_LIMIT = 12  # steps of wanting to move but not moving before sidestepping

    def __init__(self, hetero_env):
        self.env = hetero_env
        grid_size = hetero_env.unwrapped.grid_size
        self.width, self.height = int(grid_size[1]), int(grid_size[0])

        self.shelf_home = {}   # shelf key -> (x, y) home cell
        self._ensure_homes()
        self.reset_episode()

    # ------------------------------------------------------------------
    # Bookkeeping
    # ------------------------------------------------------------------

    def reset_episode(self):
        n = self.env.unwrapped.n_agents
        self._last_pos = [None] * n
        self._stuck = [0] * n
        self._wanted_move = [False] * n
        self._side_target = [None] * n
        self._t = 0

    @staticmethod
    def _shelf_key(shelf):
        sid = getattr(shelf, "id", None)
        return int(sid) if sid is not None else id(shelf)

    def _ensure_homes(self):
        """Remember where every shelf lives when it is not being carried."""
        ua = self.env.unwrapped
        carried = {a.carrying_shelf for a in ua.agents if a.carrying_shelf is not None}
        for s in ua.shelfs:
            key = self._shelf_key(s)
            if key not in self.shelf_home and s not in carried:
                self.shelf_home[key] = (int(s.x), int(s.y))

    # ------------------------------------------------------------------
    # A*
    # ------------------------------------------------------------------

    def _neighbors(self, cell, blocked):
        x, y = cell
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height and (nx, ny) not in blocked:
                yield (nx, ny)

    def astar(self, start, goal, blocked):
        if start == goal:
            return []

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in self._neighbors(current, blocked):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    # ------------------------------------------------------------------
    # Action conversion
    # ------------------------------------------------------------------

    def _turn_action(self, current_dir, desired_dir):
        """Shortest rotation from current_dir to desired_dir using the
        clockwise ordering UP -> RIGHT -> DOWN -> LEFT -> UP."""
        cur_idx = self.CLOCKWISE_ORDER.index(current_dir)
        des_idx = self.CLOCKWISE_ORDER.index(desired_dir)
        diff = (des_idx - cur_idx) % 4

        if diff == 1:
            return self.RIGHT
        elif diff == 3:
            return self.LEFT
        elif diff == 2:
            return self.RIGHT  # 180 degree turn, arbitrarily go right first
        else:
            return self.FORWARD

    def next_cell_to_action(self, agent, next_cell):
        dx = next_cell[0] - int(agent.x)
        dy = next_cell[1] - int(agent.y)

        if dx == 0 and dy == 0:
            return self.NOOP

        if dx == 1:
            desired_dir = self.RIGHT_DIR
        elif dx == -1:
            desired_dir = self.LEFT_DIR
        elif dy == 1:
            desired_dir = self.DOWN
        else:
            desired_dir = self.UP

        current_dir = agent.dir.value if hasattr(agent.dir, "value") else agent.dir

        if current_dir == desired_dir:
            return self.FORWARD

        return self._turn_action(current_dir, desired_dir)

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def get_actions_for_pursuit(self, obs_batch, pursue_flags, nearest_task_per_robot):
        ua = self.env.unwrapped
        agents = ua.agents
        n_robots = len(agents)
        self._t += 1
        self._ensure_homes()

        actions = [self.NOOP] * n_robots
        positions = [(int(a.x), int(a.y)) for a in agents]
        shelf_cells = {(int(s.x), int(s.y)) for s in ua.shelfs}
        request_queue = list(ua.request_queue)
        goals = [(int(g[0]), int(g[1])) for g in ua.goals]

        # ---- update stuck counters from last step's intent ----
        for i in range(n_robots):
            moved = self._last_pos[i] != positions[i]
            if moved:
                self._side_target[i] = None
            if self._wanted_move[i] and not moved:
                self._stuck[i] += 1
            else:
                self._stuck[i] = 0
            self._last_pos[i] = positions[i]
            self._wanted_move[i] = False

        goals_per_robot = [None] * n_robots
        toggle_per_robot = [False] * n_robots

        for i, agent in enumerate(agents):
            carrying = agent.carrying_shelf
            pos = positions[i]

            if carrying is not None:
                if carrying in request_queue:
                    if goals:
                        goals_per_robot[i] = min(goals, key=lambda g: abs(g[0] - pos[0]) + abs(g[1] - pos[1]))
                        toggle_per_robot[i] = False
                else:
                    home = self.shelf_home.get(self._shelf_key(carrying))
                    if home is not None:
                        goals_per_robot[i] = home
                        toggle_per_robot[i] = True
            else:
                if pursue_flags[i] and nearest_task_per_robot[i] is not None:
                    task = nearest_task_per_robot[i]
                    goals_per_robot[i] = (int(task[0]), int(task[1]))
                    toggle_per_robot[i] = True

        # Priority order: (0) delivering, (1) returning shelf, (2) pursuing task, (3) idle
        def robot_priority(i):
            agent = agents[i]
            if agent.carrying_shelf is not None:
                if agent.carrying_shelf in request_queue:
                    return (0, i)
                return (1, i)
            if pursue_flags[i]:
                return (2, i)
            return (3, i)

        order = sorted(range(n_robots), key=robot_priority)
        proposed_next_cell = {}

        for i in order:
            agent = agents[i]
            pos = positions[i]
            goal = goals_per_robot[i]
            carrying = agent.carrying_shelf

            if goal is not None:
                if pos == goal:
                    if toggle_per_robot[i]:
                        actions[i] = self.TOGGLE_LOAD
                    continue

                self._wanted_move[i] = True
                static_blocked = (shelf_cells - {pos, goal}) if carrying is not None else set()

                dynamic_blocked = set()
                for j in range(n_robots):
                    if j == i:
                        continue
                    if j in proposed_next_cell:
                        dynamic_blocked.add(proposed_next_cell[j])
                    else:
                        dynamic_blocked.add(positions[j])

                # Stuck-breaker sidestep
                if self._stuck[i] >= self.STUCK_LIMIT:
                    free_neighbors = list(self._neighbors(pos, static_blocked | dynamic_blocked))
                    if free_neighbors:
                        tgt = self._side_target[i]
                        if tgt not in free_neighbors:
                            tgt = free_neighbors[(i + self._t) % len(free_neighbors)]
                            self._side_target[i] = tgt
                        proposed_next_cell[i] = tgt
                        continue

                path = self.astar(pos, goal, static_blocked | dynamic_blocked)
                if not path:
                    # Next cell is occupied: if blocked for multiple steps, sidestep into free neighbor
                    raw_path = self.astar(pos, goal, static_blocked)
                    if raw_path:
                        free_n = list(self._neighbors(pos, static_blocked | dynamic_blocked))
                        if free_n and self._stuck[i] >= 2:
                            proposed_next_cell[i] = free_n[0]
                else:
                    proposed_next_cell[i] = path[0]

            else:
                # Idle robot yielding: if in transit corridor (col 0, 3, 6, 9) and blocking someone
                is_blocking = any(target == pos for target in proposed_next_cell.values())
                in_corridor = pos[0] in [0, 3, 6, 9]

                if is_blocking or (in_corridor and self._t % 3 == 0):
                    free_n = list(self._neighbors(pos, set(positions) | set(proposed_next_cell.values())))
                    if free_n:
                        # Unloaded idle robots can step into shelf cells to clear the aisle!
                        shelf_neighbors = [c for c in free_n if c in shelf_cells]
                        if shelf_neighbors:
                            proposed_next_cell[i] = shelf_neighbors[0]
                        else:
                            proposed_next_cell[i] = free_n[0]

        # Same-cell conflicts: lower robot_id wins
        cell_claims = {}
        for i, cell in proposed_next_cell.items():
            if cell not in cell_claims or i < cell_claims[cell]:
                cell_claims[cell] = i

        for i, cell in proposed_next_cell.items():
            if cell_claims[cell] == i:
                actions[i] = self.next_cell_to_action(agents[i], cell)
            else:
                actions[i] = self.NOOP

        return actions