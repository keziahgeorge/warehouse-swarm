"""
Stage 2 of Phase 4 (bidding + claims) + eligibility fix + Phase 6 Stage 2
integration: stale claims from robots believed to have failed are cleared,
freeing up their claimed tasks for other robots to bid on again.
"""

import numpy as np
import torch
from policy_network import CapabilityConditionedPolicy


class DecentralizedAuctionLayer:
    COMM_RANGE = 8

    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt"):
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

        self.bid_model = CapabilityConditionedPolicy(obs_dim=80)
        self.bid_model.load_state_dict(torch.load(bid_model_path))
        self.bid_model.eval()

        self.belief = [dict() for _ in range(self.n_robots)]
        self.known_claims = [dict() for _ in range(self.n_robots)]

        self.bid_broadcasts_made = 0
        self.bid_broadcasts_useful = 0
        self.claim_broadcasts_made = 0
        self.claim_broadcasts_useful = 0

        # Phase 6 additions
        self.stale_claims_cleared = 0
        self.stale_beliefs_cleared = 0

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

    def clear_stale_claims_from_failed_robots(self, belief_failed):
        """
        Phase 6 Stage 2: for every robot j, drop any belief/claim entry that
        was made by a robot j currently believes has failed. This frees up
        tasks previously claimed/bid-won by a now-failed robot, so others
        can bid on them again. belief_failed is the (n_robots x n_robots)
        matrix from FaultLayer.step(): belief_failed[j][k] == True means
        robot j believes robot k has failed.
        """
        if belief_failed is None:
            return

        for j in range(self.n_robots):
            # clear stale entries in belief table (bid winners)
            stale_tasks_belief = [
                task for task, (bid_val, winner) in self.belief[j].items()
                if winner != j and belief_failed[j][winner]
            ]
            for task in stale_tasks_belief:
                del self.belief[j][task]
                self.stale_beliefs_cleared += 1

            # clear stale entries in known_claims (claimed tasks)
            stale_tasks_claims = [
                task for task, claimer in self.known_claims[j].items()
                if claimer != j and belief_failed[j][claimer]
            ]
            for task in stale_tasks_claims:
                del self.known_claims[j][task]
                self.stale_claims_cleared += 1

    def step(self, obs_batch, belief_failed=None):
        """
        belief_failed: optional (n_robots x n_robots) matrix from FaultLayer.
        If provided, stale claims/beliefs from failed robots are cleared
        before this step's auction logic runs.
        """
        if belief_failed is not None:
            self.clear_stale_claims_from_failed_robots(belief_failed)

        agents = self.env.unwrapped.agents
        bids = self.compute_bids(obs_batch)

        nearest_task_per_robot = [
            self._nearest_eligible_task(agents[i], self.env.robot_types[i])
            for i in range(self.n_robots)
        ]

        # skip bidding/pursuing entirely for robots believed to have failed
        # by themselves is meaningless (a failed robot doesn't act), but we
        # also should not let a failed robot's OWN bid count if it's still
        # being computed -- handled naturally since failed robots get NOOP
        # actions from FaultLayer regardless of what's decided here.

        for i, agent_i in enumerate(agents):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue

            self.bid_broadcasts_made += 1
            changed_someone = False

            for j, agent_j in enumerate(agents):
                if i == j or self._distance(agent_i, agent_j) > self.COMM_RANGE:
                    continue

                current_best = self.belief[j].get(task_i)
                if current_best is None or bids[i] > current_best[0]:
                    old_winner = current_best[1] if current_best else None
                    self.belief[j][task_i] = (bids[i], i)
                    if old_winner != i:
                        changed_someone = True

            if changed_someone:
                self.bid_broadcasts_useful += 1

        for i in range(self.n_robots):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            current_best = self.belief[i].get(task_i)
            if current_best is None or bids[i] > current_best[0]:
                self.belief[i][task_i] = (bids[i], i)

        provisional_pursue = []
        for i in range(self.n_robots):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                provisional_pursue.append(False)
                continue
            best_bid, best_robot = self.belief[i][task_i]
            provisional_pursue.append(best_robot == i)

        for i, agent_i in enumerate(agents):
            task_i = nearest_task_per_robot[i]
            if task_i is None or not provisional_pursue[i]:
                continue

            self.claim_broadcasts_made += 1
            changed_someone = False

            for j, agent_j in enumerate(agents):
                if i == j or self._distance(agent_i, agent_j) > self.COMM_RANGE:
                    continue

                would_have_pursued = (
                    nearest_task_per_robot[j] == task_i and provisional_pursue[j]
                )
                self.known_claims[j][task_i] = i

                if would_have_pursued and j != i:
                    changed_someone = True

            if changed_someone:
                self.claim_broadcasts_useful += 1

        for i in range(self.n_robots):
            if provisional_pursue[i] and nearest_task_per_robot[i] is not None:
                self.known_claims[i][nearest_task_per_robot[i]] = i

        final_pursue = []
        for i in range(self.n_robots):
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                final_pursue.append(False)
                continue

            claimer = self.known_claims[i].get(task_i)
            if claimer is not None and claimer != i:
                final_pursue.append(False)
            else:
                final_pursue.append(provisional_pursue[i])

        return final_pursue, bids, nearest_task_per_robot

    def clear_stale_beliefs(self):
        active_tasks = set(self._task_id(s) for s in self.env.unwrapped.request_queue)
        for i in range(self.n_robots):
            self.belief[i] = {t: v for t, v in self.belief[i].items() if t in active_tasks}
            self.known_claims[i] = {t: v for t, v in self.known_claims[i].items() if t in active_tasks}

    def report_stats(self):
        bid_rate = (self.bid_broadcasts_useful / self.bid_broadcasts_made * 100) if self.bid_broadcasts_made else 0
        claim_rate = (self.claim_broadcasts_useful / self.claim_broadcasts_made * 100) if self.claim_broadcasts_made else 0
        print(f"Bid broadcasts made: {self.bid_broadcasts_made}, "
              f"changed a receiver's belief: {self.bid_broadcasts_useful} ({bid_rate:.1f}%)")
        print(f"Claim broadcasts made: {self.claim_broadcasts_made}, "
              f"prevented a would-be conflict: {self.claim_broadcasts_useful} ({claim_rate:.1f}%)")
        print(f"Stale beliefs cleared (from failed robots): {self.stale_beliefs_cleared}")
        print(f"Stale claims cleared (from failed robots): {self.stale_claims_cleared}")