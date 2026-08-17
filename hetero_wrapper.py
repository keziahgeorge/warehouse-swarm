import gymnasium as gym
import numpy as np


class HeterogeneousWarehouseWrapper(gym.Wrapper):
    """
    Wraps RWARE to add:
      - robot capability types (fast_light, heavy_load, balanced)
      - battery draining per type
      - speed effects per type (bonus/dropped moves)
      - task types on requested shelves (urgent_light, heavy_shelf, standard_delivery)
      - nearest-task info appended to each robot's observation
      - capability mismatch penalty on delivery
      - distance-based shaping reward (addresses sparse-reward problem)
      - match/mismatch counters for evaluation
    """

    TYPE_NAMES = ["fast_light", "heavy_load", "balanced"]
    TASK_NAMES = ["urgent_light", "heavy_shelf", "standard_delivery"]
    TASK_WEIGHT = {0: 1, 1: 3, 2: 2}

    BATTERY_DRAIN = {0: 0.8, 1: 1.3, 2: 1.0}
    BONUS_STEP_PROB = {0: 0.5, 1: 0.0, 2: 0.0}
    DROP_STEP_PROB = {0: 0.0, 1: 0.3, 2: 0.0}

    ACCEPTABLE_ROBOTS = {
        0: {0, 2},      # urgent_light: fast_light or balanced
        1: {1},         # heavy_shelf: heavy_load only
        2: {0, 1, 2},   # standard_delivery: anyone
    }
    MISMATCH_PENALTY = 0.5

    MAX_BATTERY = 100.0
    FORWARD_ACTION = 1
    TOGGLE_LOAD_ACTION = 4
    MAX_SEARCH_DISTANCE = 20.0

    SHAPING_SCALE = 0.05

    def __init__(self, env, robot_types=None, seed=None, verbose=False):
        super().__init__(env)
        n_agents = self.unwrapped.n_agents

        if robot_types is None:
            robot_types = [i % 3 for i in range(n_agents)]
        assert len(robot_types) == n_agents

        self.robot_types = robot_types
        self.type_embeddings = np.eye(3)
        self.task_embeddings = np.eye(3)
        self.battery = np.full(n_agents, self.MAX_BATTERY, dtype=np.float32)
        self.verbose = verbose
        self.rng = np.random.default_rng(seed)

        self.shelf_task_type = {}
        self._prev_request_queue = []
        self._prev_objective_distance = [None] * n_agents

        # --- delivery tracking counters, per robot ---
        self.match_count = [0] * n_agents
        self.mismatch_count = [0] * n_agents
        # also track, per robot, how many deliveries of EACH task type they did
        self.deliveries_by_task_type = [{0: 0, 1: 0, 2: 0} for _ in range(n_agents)]

        print("Robot types assigned:")
        for i, t in enumerate(self.robot_types):
            print(f"  Robot {i}: {self.TYPE_NAMES[t]} "
                  f"(battery drain: {self.BATTERY_DRAIN[t]}, "
                  f"bonus prob: {self.BONUS_STEP_PROB[t]}, "
                  f"drop prob: {self.DROP_STEP_PROB[t]})")

    def _assign_task_types_to_new_requests(self):
        current_queue = list(self.unwrapped.request_queue)
        for shelf in current_queue:
            if shelf not in self.shelf_task_type:
                task_type = int(self.rng.integers(0, 3))
                self.shelf_task_type[shelf] = task_type
                if self.verbose:
                    print(f"  New task requested: shelf at ({shelf.x},{shelf.y}) "
                          f"-> type={self.TASK_NAMES[task_type]}")
        for shelf in list(self.shelf_task_type.keys()):
            if shelf not in current_queue:
                del self.shelf_task_type[shelf]
        self._prev_request_queue = current_queue

    def _get_nearest_task_info(self, agent):
        queue = self.unwrapped.request_queue
        if not queue:
            return np.zeros(5, dtype=np.float32)

        best_dist = None
        best_shelf = None
        for shelf in queue:
            dist = abs(shelf.x - agent.x) + abs(shelf.y - agent.y)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_shelf = shelf

        task_type = self.shelf_task_type.get(best_shelf, 2)
        type_onehot = self.task_embeddings[task_type]
        weight_norm = np.array([self.TASK_WEIGHT[task_type] / 3.0], dtype=np.float32)
        dist_norm = np.array([min(best_dist / self.MAX_SEARCH_DISTANCE, 1.0)], dtype=np.float32)

        return np.concatenate([type_onehot, weight_norm, dist_norm]).astype(np.float32)

    def _get_objective_distance(self, agent):
        if agent.carrying_shelf is not None:
            goals = self.unwrapped.goals
            if not goals:
                return None
            dists = [abs(gx - agent.x) + abs(gy - agent.y) for gx, gy in goals]
            return min(dists)
        else:
            queue = self.unwrapped.request_queue
            if not queue:
                return None
            dists = [abs(s.x - agent.x) + abs(s.y - agent.y) for s in queue]
            return min(dists)

    def _augment_obs(self, obs_tuple):
        new_obs = []
        for i, obs in enumerate(obs_tuple):
            type_vec = self.type_embeddings[self.robot_types[i]]
            battery_norm = np.array([self.battery[i] / self.MAX_BATTERY], dtype=np.float32)
            task_info = self._get_nearest_task_info(self.unwrapped.agents[i])
            augmented = np.concatenate([obs, type_vec, battery_norm, task_info]).astype(np.float32)
            new_obs.append(augmented)
        return tuple(new_obs)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.battery[:] = self.MAX_BATTERY
        self.shelf_task_type = {}
        self._assign_task_types_to_new_requests()

        self._prev_objective_distance = [
            self._get_objective_distance(agent) for agent in self.unwrapped.agents
        ]

        return self._augment_obs(obs), info

    def _apply_speed_effects(self, actions):
        actions = list(actions)
        bonus_flags = [False] * len(actions)
        for i, t in enumerate(self.robot_types):
            agent_action = actions[i][0] if isinstance(actions[i], tuple) else actions[i]
            if agent_action == self.FORWARD_ACTION:
                if self.DROP_STEP_PROB[t] > 0 and self.rng.random() < self.DROP_STEP_PROB[t]:
                    if isinstance(actions[i], tuple):
                        actions[i] = (0,) + actions[i][1:]
                    else:
                        actions[i] = 0
                elif self.BONUS_STEP_PROB[t] > 0 and self.rng.random() < self.BONUS_STEP_PROB[t]:
                    bonus_flags[i] = True
        return actions, bonus_flags

    def step(self, actions):
        modified_actions, bonus_flags = self._apply_speed_effects(actions)

        queue_before = list(self.unwrapped.request_queue)
        shelf_task_before = dict(self.shelf_task_type)

        obs, rewards, terminated, truncated, info = self.env.step(modified_actions)

        if any(bonus_flags):
            bonus_actions = []
            for i in range(len(modified_actions)):
                if bonus_flags[i]:
                    if isinstance(modified_actions[i], tuple):
                        bonus_actions.append((self.FORWARD_ACTION,) + modified_actions[i][1:])
                    else:
                        bonus_actions.append(self.FORWARD_ACTION)
                else:
                    if isinstance(modified_actions[i], tuple):
                        bonus_actions.append((0,) + modified_actions[i][1:])
                    else:
                        bonus_actions.append(0)
            obs, bonus_rewards, terminated, truncated, info = self.env.step(bonus_actions)
            rewards = [r1 + r2 for r1, r2 in zip(rewards, bonus_rewards)]

        rewards = list(rewards)

        queue_after = set(self.unwrapped.request_queue)
        delivered_shelves = [s for s in queue_before if s not in queue_after]

        for shelf in delivered_shelves:
            task_type = shelf_task_before.get(shelf, 2)
            for i, agent in enumerate(self.unwrapped.agents):
                if agent.x == shelf.x and agent.y == shelf.y:
                    self.deliveries_by_task_type[i][task_type] += 1
                    if self.robot_types[i] not in self.ACCEPTABLE_ROBOTS[task_type]:
                        rewards[i] -= self.MISMATCH_PENALTY
                        self.mismatch_count[i] += 1
                        if self.verbose:
                            print(f"  MISMATCH: Robot {i} ({self.TYPE_NAMES[self.robot_types[i]]}) "
                                  f"delivered a {self.TASK_NAMES[task_type]} task -> penalty applied")
                    else:
                        self.match_count[i] += 1
                        if self.verbose:
                            print(f"  OK MATCH: Robot {i} ({self.TYPE_NAMES[self.robot_types[i]]}) "
                                  f"delivered a {self.TASK_NAMES[task_type]} task -> no penalty")

        for i, agent in enumerate(self.unwrapped.agents):
            new_dist = self._get_objective_distance(agent)
            old_dist = self._prev_objective_distance[i]

            if new_dist is not None and old_dist is not None:
                delta = old_dist - new_dist
                rewards[i] += self.SHAPING_SCALE * delta

            self._prev_objective_distance[i] = new_dist

        for i, t in enumerate(self.robot_types):
            self.battery[i] = max(0.0, self.battery[i] - self.BATTERY_DRAIN[t])

        self._assign_task_types_to_new_requests()

        return self._augment_obs(obs), rewards, terminated, truncated, info