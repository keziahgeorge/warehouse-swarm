"""
Phase 7 ablation: no-communication auction layer.
Identical to DecentralizedAuctionLayer, except COMM_RANGE = 0, meaning no
robot ever hears another robot's bid or claim broadcast. Each robot only
ever "hears" itself, so every robot pursues its own nearest eligible task
independently with no conflict resolution and no fault-driven reallocation
possible (since a robot can never learn about another robot's failure
either, this file also skips fault-layer integration entirely -- there is
no mechanism BY WHICH it could reallocate, which is exactly the point of
this ablation).

Used to isolate how much of Phase 6's fault-tolerance result depends on
the communication/reallocation mechanism, versus the system just doing
fine on its own regardless.
"""

import torch
from policy_network import CapabilityConditionedPolicy


class NoCommAuctionLayer:
    COMM_RANGE = 0  # nobody ever hears anybody

    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt"):
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

        self.bid_model = CapabilityConditionedPolicy(obs_dim=80)
        self.bid_model.load_state_dict(torch.load(bid_model_path))
        self.bid_model.eval()

    def _task_id(self, shelf):
        return (shelf.x, shelf.y)

    def _distance(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def compute_bids(self, obs_batch):
        bids = []
        with torch.no_grad():
            for i in range(self.n_robots):
                obs_tensor = torch.tensor(obs_batch[i], dtype=torch.float32).unsqueeze(0)
                bid_value, _ = self.bid_model(obs_tensor)
                bids.append(bid_value.item())
        return bids

    def _nearest_eligible_task(self, agent, robot_type):
        queue = self.env.unwrapped.request_queue
        if not queue:
            return None

        best_dist, best_shelf = None, None
        for shelf in queue:
            task_type = self.env.shelf_task_type.get(shelf, 2)
            if robot_type not in self.env.ACCEPTABLE_ROBOTS[task_type]:
                continue
            d = self._distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf

        if best_shelf is None:
            return None
        return self._task_id(best_shelf)

    def step(self, obs_batch, belief_failed=None):
        """
        No communication: every robot independently decides to pursue its
        own nearest eligible task, with no awareness of what any other
        robot is doing, no conflict resolution, and no ability to detect
        or react to failures (belief_failed is accepted for interface
        compatibility with the test harness but intentionally ignored --
        a robot with COMM_RANGE=0 cannot hear heartbeats either).
        """
        agents = self.env.unwrapped.agents
        bids = self.compute_bids(obs_batch)  # computed but unused for coordination

        nearest_task_per_robot = [
            self._nearest_eligible_task(agents[i], self.env.robot_types[i])
            for i in range(self.n_robots)
        ]

        # every robot with an eligible task simply pursues it -- no
        # awareness of collisions or duplicate pursuit of the same shelf
        pursue = [task is not None for task in nearest_task_per_robot]

        return pursue, bids, nearest_task_per_robot

    def clear_stale_beliefs(self):
        pass  # no belief tables exist in this ablation

    def report_stats(self):
        print("No-comm ablation: no broadcast/belief statistics to report (by design).")