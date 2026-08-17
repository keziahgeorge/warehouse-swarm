"""
Phase 5: basic A* pathfinding + RWARE action conversion, with simple
decentralized collision avoidance (lower robot_id has priority on
contested next-cells).

Fixes from previous version:
  - Added TOGGLE_LOAD when the robot has arrived at its target (shelf or goal)
  - Fixed turn_map to correctly handle all 4 directions with proper
    left/right rotation logic instead of only covering 4 of 12 cases
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

    def __init__(self, hetero_env):
        self.env = hetero_env
        grid_size = hetero_env.unwrapped.grid_size
        self.width, self.height = grid_size[1], grid_size[0]

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
            return self.FORWARD  # diff == 0, already facing correctly (shouldn't hit this path)

    def next_cell_to_action(self, agent, next_cell):
        dx = next_cell[0] - agent.x
        dy = next_cell[1] - agent.y

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

    def get_actions_for_pursuit(self, obs_batch, pursue_flags, nearest_task_per_robot):
        agents = self.env.unwrapped.agents
        n_robots = len(agents)
        actions = [self.NOOP] * n_robots
        proposed_next_cell = {}

        blocked = set()

        for i, agent in enumerate(agents):
            if not pursue_flags[i]:
                continue

            if agent.carrying_shelf is not None:
                goals = self.env.unwrapped.goals
                if not goals:
                    continue
                goal = min(goals, key=lambda g: abs(g[0] - agent.x) + abs(g[1] - agent.y))
            else:
                task_id = nearest_task_per_robot[i]
                if task_id is None:
                    continue
                goal = task_id

            # arrived at target -- toggle load (pick up or drop off)
            if (agent.x, agent.y) == goal:
                actions[i] = self.TOGGLE_LOAD
                continue

            path = self.astar((agent.x, agent.y), goal, blocked)
            if not path:
                continue

            proposed_next_cell[i] = path[0]

        cell_claims = {}
        for i, cell in proposed_next_cell.items():
            if cell not in cell_claims or i < cell_claims[cell]:
                cell_claims[cell] = i

        for i, cell in proposed_next_cell.items():
            if cell_claims[cell] == i:
                agent = agents[i]
                actions[i] = self.next_cell_to_action(agent, cell)
            else:
                actions[i] = self.NOOP

        return actions