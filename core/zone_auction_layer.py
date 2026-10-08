"""
Phase 8 (patched): Zone-Aware Auction Layer

Changes over the previous version:
  1. Task ownership comes from the coordinator's routing map
     (task_zone_map), so cross-zone handoffs really change who may bid.
     If a task is not in the map, the spatial zone is used.
  2. Robots that are carrying a shelf do not bid (they are busy), and
     shelves being carried are not open tasks for anyone else.
  3. Failed robots are SILENT: they send no bids or claims. (Before, a
     dead robot kept re-asserting its claims every step, undoing the
     stale-claim clearing.) Shelves under a dead robot are skipped.
  4. Bids are computed in one batched forward pass; the coordinator can
     pass pre-computed bids so this is done once per step, not per zone.
  5. Bid timeout now fires when NO LIVE ROBOT IN THE ZONE IS CAPABLE of a
     task (e.g. an H task in a zone without a heavy_load robot), instead
     of when no robot happened to currently bid on it.
  6. reset_episode() clears all tables when RWARE resets the world.
"""

import os
from pathlib import Path
import numpy as np
import torch
from policy_network import CapabilityConditionedPolicy


def _resolve_model_path(path: str) -> str:
    if os.path.exists(path):
        return path
    core_candidate = Path(__file__).resolve().parent / path
    if core_candidate.exists():
        return str(core_candidate)
    models_candidate = Path(__file__).resolve().parent.parent / "models_and_data" / path
    if models_candidate.exists():
        return str(models_candidate)
    return path


class ZoneAuctionLayer:
    """
    Zone-scoped decentralized auction layer.

    Parameters
    ----------
    hetero_env       : HeterogeneousWarehouseWrapper
    zone_id          : int   - which zone this auction instance manages
    zone_assignment  : dict  - {robot_id -> zone_id} from ZonePartitioner
    zone_partitioner : ZonePartitioner
    bid_model_path   : str   - path to trained bid head weights
    task_zone_map    : dict  - {task_id -> owning zone_id}, shared with and
                               mutated in place by HierarchicalCoordinator
    """

    COMM_RANGE = 8
    BID_TIMEOUT = 10  # steps a task may be uncovered before it is handed off

    def __init__(self, hetero_env, zone_id: int, zone_assignment: dict,
                 zone_partitioner, bid_model_path="bid_head_trained.pt",
                 task_zone_map=None):
        self.env = hetero_env
        self.zone_id = zone_id
        self.zone_assignment = zone_assignment
        self.partitioner = zone_partitioner
        self.task_zone_map = task_zone_map if task_zone_map is not None else {}

        self.n_robots = hetero_env.unwrapped.n_agents
        self.zone_robots = [i for i, z in zone_assignment.items() if z == zone_id]

        self.bid_model = CapabilityConditionedPolicy(obs_dim=80)
        resolved_path = _resolve_model_path(bid_model_path)
        self.bid_model.load_state_dict(torch.load(resolved_path, map_location="cpu"))
        self.bid_model.eval()

        self.belief = [dict() for _ in range(self.n_robots)]
        self.known_claims = [dict() for _ in range(self.n_robots)]

        # --- stats ---
        self.bid_broadcasts_made = 0
        self.bid_broadcasts_useful = 0
        self.claim_broadcasts_made = 0
        self.claim_broadcasts_useful = 0
        self.stale_claims_cleared = 0
        self.stale_beliefs_cleared = 0
        self.comm_events_per_step: list = []

        # task_id -> consecutive steps the task has had no capable robot here
        self._no_bid_counter: dict = {}
        self.timed_out_tasks: list = []

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _task_id(self, shelf):
        return (int(shelf.x), int(shelf.y))

    def _distance(self, a, b):
        return abs(int(a.x) - int(b.x)) + abs(int(a.y) - int(b.y))

    def _task_in_zone(self, shelf) -> bool:
        """True if this zone currently OWNS the task (routing map first,
        spatial zone as fallback)."""
        owner = self.task_zone_map.get(self._task_id(shelf))
        if owner is None:
            owner = self.partitioner.get_zone(int(shelf.x), int(shelf.y))
        return owner == self.zone_id

    def _carried_shelves(self):
        return {a.carrying_shelf for a in self.env.unwrapped.agents
                if a.carrying_shelf is not None}

    def _nearest_eligible_task(self, agent, robot_type, carried, dead_cells, excluded=None):
        """
        Nearest open task that:
          - this zone owns,
          - is not currently being carried by someone,
          - does not sit under a dead robot,
          - this robot type is eligible for,
          - is not already in the excluded set (e.g. chosen by a higher-priority peer).
        Busy (carrying) robots do not bid at all.
        """
        if agent.carrying_shelf is not None:
            return None

        queue = self.env.unwrapped.request_queue
        best_dist, best_shelf = None, None
        for shelf in queue:
            tid = self._task_id(shelf)
            if excluded and tid in excluded:
                continue
            if shelf in carried:
                continue
            if (int(shelf.x), int(shelf.y)) in dead_cells:
                continue
            if not self._task_in_zone(shelf):
                continue
            task_type = self.env.shelf_task_type.get(shelf, 2)
            if robot_type not in self.env.ACCEPTABLE_ROBOTS[task_type]:
                continue
            d = self._distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf

        if best_shelf is None:
            return None
        return self._task_id(best_shelf)

    def compute_bids(self, obs_batch):
        """One batched forward pass for all robots."""
        with torch.no_grad():
            x = torch.as_tensor(np.asarray(obs_batch, dtype=np.float32))
            bid_value, _ = self.bid_model(x)
        return bid_value.squeeze(1).tolist()

    # ------------------------------------------------------------------
    # Stale-claim clearing
    # ------------------------------------------------------------------

    def clear_stale_claims_from_failed_robots(self, belief_failed):
        if belief_failed is None:
            return
        for j in self.zone_robots:
            stale_belief = [
                task for task, (bid_val, winner) in self.belief[j].items()
                if winner != j and belief_failed[j][winner]
            ]
            for task in stale_belief:
                del self.belief[j][task]
                self.stale_beliefs_cleared += 1

            stale_claims = [
                task for task, claimer in self.known_claims[j].items()
                if claimer != j and belief_failed[j][claimer]
            ]
            for task in stale_claims:
                del self.known_claims[j][task]
                self.stale_claims_cleared += 1

    def clear_stale_beliefs(self):
        """Remove belief/claim entries for tasks no longer open."""
        carried = self._carried_shelves()
        active_tasks = {
            self._task_id(s) for s in self.env.unwrapped.request_queue
            if s not in carried
        }
        for i in self.zone_robots:
            self.belief[i] = {
                t: v for t, v in self.belief[i].items() if t in active_tasks
            }
            self.known_claims[i] = {
                t: v for t, v in self.known_claims[i].items() if t in active_tasks
            }

    def reset_episode(self):
        """Call when RWARE resets the world."""
        self.belief = [dict() for _ in range(self.n_robots)]
        self.known_claims = [dict() for _ in range(self.n_robots)]
        self._no_bid_counter = {}
        self.timed_out_tasks = []

    # ------------------------------------------------------------------
    # Bid-timeout tracking for cross-zone handoff
    # ------------------------------------------------------------------

    def _update_bid_timeout(self, zone_task_ids: set, covered_task_ids: set):
        """
        zone_task_ids    : open tasks currently owned by this zone
        covered_task_ids : subset that at least one LIVE zone robot is
                           capable of serving
        A task that stays uncovered for BID_TIMEOUT steps is emitted into
        self.timed_out_tasks for the coordinator to hand off.
        """
        self.timed_out_tasks = []

        for tid in zone_task_ids:
            if tid in covered_task_ids:
                self._no_bid_counter[tid] = 0
            else:
                self._no_bid_counter[tid] = self._no_bid_counter.get(tid, 0) + 1

        for tid, count in list(self._no_bid_counter.items()):
            if count >= self.BID_TIMEOUT:
                self.timed_out_tasks.append(tid)
                del self._no_bid_counter[tid]

        for tid in list(self._no_bid_counter.keys()):
            if tid not in zone_task_ids:
                del self._no_bid_counter[tid]

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, obs_batch, belief_failed=None, bids=None, failed=None):
        """
        Run one auction step for this zone's robots.

        Parameters
        ----------
        obs_batch    : np.ndarray (n_robots, obs_dim)
        belief_failed: optional (n_robots x n_robots) bool matrix
        bids         : optional pre-computed bids (list of n_robots floats)
        failed       : optional list[bool] of ACTUALLY failed robots; they
                       send no messages and do not bid

        Returns
        -------
        pursue, bids, nearest_tasks  (all length n_robots)
        """
        n = self.n_robots
        failed = list(failed) if failed is not None else [False] * n

        if belief_failed is not None:
            self.clear_stale_claims_from_failed_robots(belief_failed)

        ua = self.env.unwrapped
        agents = ua.agents
        if bids is None:
            bids = self.compute_bids(obs_batch)

        active = [i for i in self.zone_robots if not failed[i]]
        carried = self._carried_shelves()
        dead_cells = {(int(agents[i].x), int(agents[i].y))
                      for i in range(n) if failed[i]}

        nearest_task_per_robot = [None] * n
        selected_tasks = set()
        # Sort by bid priority so highest bidder gets first choice of distinct tasks
        sorted_active = sorted(active, key=lambda idx: bids[idx], reverse=True)
        for i in sorted_active:
            task = self._nearest_eligible_task(
                agents[i], self.env.robot_types[i], carried, dead_cells,
                excluded=selected_tasks
            )
            nearest_task_per_robot[i] = task
            if task is not None:
                selected_tasks.add(task)

        # ---- Phase 1: bid broadcasting (zone-scoped, live robots only) ----
        step_comm_events = 0

        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue

            self.bid_broadcasts_made += 1
            changed_someone = False

            for j in active:
                if i == j or self._distance(agents[i], agents[j]) > self.COMM_RANGE:
                    continue

                step_comm_events += 1
                current_best = self.belief[j].get(task_i)
                if current_best is None or bids[i] > current_best[0]:
                    old_winner = current_best[1] if current_best else None
                    self.belief[j][task_i] = (bids[i], i)
                    if old_winner != i:
                        changed_someone = True

            if changed_someone:
                self.bid_broadcasts_useful += 1

        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            current_best = self.belief[i].get(task_i)
            if current_best is None or bids[i] > current_best[0]:
                self.belief[i][task_i] = (bids[i], i)

        # ---- Phase 2: provisional pursue ----
        provisional_pursue = [False] * n
        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            best_bid, best_robot = self.belief[i][task_i]
            provisional_pursue[i] = (best_robot == i)

        # ---- Phase 3: claim broadcasting ----
        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None or not provisional_pursue[i]:
                continue

            self.claim_broadcasts_made += 1
            changed_someone = False

            for j in active:
                if i == j or self._distance(agents[i], agents[j]) > self.COMM_RANGE:
                    continue

                step_comm_events += 1
                would_have_pursued = (
                    nearest_task_per_robot[j] == task_i and provisional_pursue[j]
                )
                self.known_claims[j][task_i] = i
                if would_have_pursued and j != i:
                    changed_someone = True

            if changed_someone:
                self.claim_broadcasts_useful += 1

        for i in active:
            if provisional_pursue[i] and nearest_task_per_robot[i] is not None:
                self.known_claims[i][nearest_task_per_robot[i]] = i

        # ---- Phase 4: final pursue (claim-resolved) ----
        final_pursue = [False] * n
        for i in active:
            task_i = nearest_task_per_robot[i]
            if task_i is None:
                continue
            claimer = self.known_claims[i].get(task_i)
            if claimer is not None and claimer != i:
                final_pursue[i] = False
            else:
                final_pursue[i] = provisional_pursue[i]

        self.comm_events_per_step.append(step_comm_events)

        # ---- Handoff bookkeeping: which owned tasks have a capable live robot? ----
        zone_tasks = {}
        for s in ua.request_queue:
            if s in carried or not self._task_in_zone(s):
                continue
            zone_tasks[self._task_id(s)] = self.env.shelf_task_type.get(s, 2)

        covered = {
            tid for tid, tt in zone_tasks.items()
            if any(self.env.robot_types[i] in self.env.ACCEPTABLE_ROBOTS[tt]
                   for i in active)
        }
        self._update_bid_timeout(set(zone_tasks.keys()), covered)

        return final_pursue, bids, nearest_task_per_robot

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def avg_comm_per_step(self) -> float:
        """Mean intra-zone comm events per step (whole zone)."""
        if not self.comm_events_per_step:
            return 0.0
        return sum(self.comm_events_per_step) / len(self.comm_events_per_step)

    def avg_comm_per_robot_per_step(self) -> float:
        n = len(self.zone_robots)
        if n == 0:
            return 0.0
        return self.avg_comm_per_step() / n

    def report_stats(self):
        bid_rate = (
            self.bid_broadcasts_useful / self.bid_broadcasts_made * 100
            if self.bid_broadcasts_made else 0
        )
        claim_rate = (
            self.claim_broadcasts_useful / self.claim_broadcasts_made * 100
            if self.claim_broadcasts_made else 0
        )
        print(f"[Zone {self.zone_id}] Robots: {self.zone_robots}")
        print(f"  Bid broadcasts: {self.bid_broadcasts_made} "
              f"({self.bid_broadcasts_useful} useful, {bid_rate:.1f}%)")
        print(f"  Claim broadcasts: {self.claim_broadcasts_made} "
              f"({self.claim_broadcasts_useful} useful, {claim_rate:.1f}%)")
        print(f"  Stale beliefs cleared: {self.stale_beliefs_cleared}, "
              f"stale claims cleared: {self.stale_claims_cleared}")
        print(f"  Avg comm events/step: {self.avg_comm_per_step():.3f} "
              f"({self.avg_comm_per_robot_per_step():.3f} per robot)")