"""
Phase 6, Stage 1: heartbeat-based failure detection.

Each active robot 'broadcasts' a heartbeat every step to robots within
COMM_RANGE (reusing the same range concept as the auction layer). If a
robot hasn't been heard from in HEARTBEAT_TIMEOUT steps, other robots
mark it as failed in their local belief -- no central registry, purely
local/decentralized, consistent with the rest of the system.

Stage 1 only detects failures; it does not yet reallocate stale claims
(that's Stage 2).
"""

import numpy as np


class FaultLayer:
    COMM_RANGE = 20
    HEARTBEAT_TIMEOUT = 15  # steps without a heartbeat before considered failed

    def __init__(self, hetero_env):
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

        # ground truth: which robots are actually failed (for evaluation only,
        # not something other robots have direct access to)
        self.actual_failed = [False] * self.n_robots

        # last_heard[i][j] = steps since robot i last heard a heartbeat from j
        self.steps_since_heard = [[0] * self.n_robots for _ in range(self.n_robots)]

        # belief_failed[i][j] = True if robot i currently believes robot j has failed
        self.belief_failed = [[False] * self.n_robots for _ in range(self.n_robots)]

    def _distance(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def inject_failure(self, robot_id):
        """Mark a robot as failed. It will stop broadcasting heartbeats from
        this point on. Call this once, at whatever step you want the failure
        to occur."""
        self.actual_failed[robot_id] = True
        print(f"[FaultLayer] Robot {robot_id} has FAILED at this step.")

    def step(self):
        """
        Call once per environment step. Active (non-failed) robots broadcast
        heartbeats to robots within range. Updates each robot's belief about
        which other robots have failed, based on heartbeat timeout.

        Returns: belief_failed (n_robots x n_robots bool matrix) -- what each
        robot currently believes about every other robot's status.
        """
        agents = self.env.unwrapped.agents

        # every robot's "steps since heard" ticks up by 1 by default
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i != j:
                    self.steps_since_heard[i][j] += 1

        # active robots broadcast heartbeat to everyone in range -> resets their counter
        for i, agent_i in enumerate(agents):
            if self.actual_failed[i]:
                continue  # failed robots don't broadcast

            for j, agent_j in enumerate(agents):
                if i == j:
                    continue
                if self._distance(agent_i, agent_j) <= self.COMM_RANGE:
                    self.steps_since_heard[j][i] = 0

        # update belief based on timeout
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i == j:
                    continue
                self.belief_failed[i][j] = self.steps_since_heard[i][j] > self.HEARTBEAT_TIMEOUT

        return self.belief_failed

    def get_actions_override(self, actions):
        """Force NOOP for any robot that has actually failed, regardless of
        what the rest of the pipeline decided. Call this right before
        vec_env.step()."""
        actions = list(actions)
        for i in range(self.n_robots):
            if self.actual_failed[i]:
                actions[i] = 0  # NOOP
        return actions

    def detection_accuracy_report(self):
        """Compare belief_failed against actual_failed for a quick sanity check."""
        print("\n=== Fault Detection Accuracy ===")
        for i in range(self.n_robots):
            beliefs_about_i = [self.belief_failed[j][i] for j in range(self.n_robots) if j != i]
            correctly_detected = all(b == self.actual_failed[i] for b in beliefs_about_i) if beliefs_about_i else None
            print(f"Robot {i}: actually_failed={self.actual_failed[i]}, "
                  f"detected as failed by others: {beliefs_about_i}")