"""
Phase 8 (patched): Hierarchical Coordinator

Changes over the previous version:
  1. The routing map (_task_zone) is shared with every ZoneAuctionLayer, so
     cross-zone handoffs actually move a task to another zone's auction.
  2. Carried shelves are not treated as new tasks (their position changes
     every step, which used to inflate total_tasks_seen).
  3. Bids are computed once per step and shared by all zones; actual
     failures are passed to the auctions so dead robots stay silent.
  4. Episode resets are handled: reset_episode() clears routing, claims and
     pathfinder state. step() also auto-detects a RWARE reset.
  5. avg_comm_per_robot_per_step() fixed: total events per step across all
     zones divided by fleet size (it was multiplied by zone size before).
  6. inject_failure() prints once.
"""

from collections import defaultdict
from zone_auction_layer import ZoneAuctionLayer
from zone_fault_layer import ZoneFaultLayer
from pathfinding import GridPathfinder
from zone_partitioner import ZonePartitioner


class HierarchicalCoordinator:
    def __init__(self, hetero_env, partitioner: ZonePartitioner,
                 zone_assignment: dict, bid_model_path="bid_head_trained.pt"):
        self.env = hetero_env
        self.partitioner = partitioner
        self.zone_assignment = zone_assignment
        self.n_zones = partitioner.n_zones
        self.n_robots = hetero_env.unwrapped.n_agents

        # --- Task routing state (must exist BEFORE the auctions are built) ---
        # task_id -> zone_id currently responsible; shared, mutated in place only
        self._task_zone: dict = {}
        self._task_tried_zones: dict = defaultdict(set)

        self.auctions = [
            ZoneAuctionLayer(
                hetero_env, zone_id=z,
                zone_assignment=zone_assignment,
                zone_partitioner=partitioner,
                bid_model_path=bid_model_path,
                task_zone_map=self._task_zone,
            )
            for z in range(self.n_zones)
        ]

        self.fault_layers = [
            ZoneFaultLayer(hetero_env, zone_assignment=zone_assignment)
            for _ in range(self.n_zones)
        ]

        self.pathfinder = GridPathfinder(hetero_env)

        # --- Stats ---
        self.cross_zone_handoffs = 0
        self.tasks_routed_by_zone = defaultdict(int)
        self.total_tasks_seen = 0

        self._last_cur_steps = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _carried_shelves(self):
        return {a.carrying_shelf for a in self.env.unwrapped.agents
                if a.carrying_shelf is not None}

    def _open_tasks(self):
        """Requested shelves that nobody is carrying yet."""
        carried = self._carried_shelves()
        return [s for s in self.env.unwrapped.request_queue if s not in carried]

    # ------------------------------------------------------------------
    # Task routing
    # ------------------------------------------------------------------

    def _assign_new_tasks(self):
        for shelf in self._open_tasks():
            tid = (int(shelf.x), int(shelf.y))
            if tid not in self._task_zone:
                sorted_zones = self.partitioner.nearest_zones_to_task(shelf.x, shelf.y)
                nearest = sorted_zones[0]
                self._task_zone[tid] = nearest
                self._task_tried_zones[tid].add(nearest)
                self.tasks_routed_by_zone[nearest] += 1
                self.total_tasks_seen += 1

    def _handle_bid_timeouts(self):
        """Tasks no live robot of their zone can serve move to the next zone."""
        for z in range(self.n_zones):
            for tid in self.auctions[z].timed_out_tasks:
                if tid not in self._task_zone:
                    continue
                sorted_zones = self.partitioner.nearest_zones_to_task(tid[0], tid[1])
                already_tried = self._task_tried_zones[tid]
                next_zone = None
                for candidate in sorted_zones:
                    if candidate not in already_tried:
                        next_zone = candidate
                        break

                if next_zone is None:
                    # every zone tried once: start another lap from the current owner
                    self._task_tried_zones[tid] = {self._task_zone[tid]}
                    continue

                self._task_zone[tid] = next_zone
                self._task_tried_zones[tid].add(next_zone)
                self.cross_zone_handoffs += 1

    def _clean_completed_tasks(self):
        active = {(int(s.x), int(s.y)) for s in self._open_tasks()}
        for tid in list(self._task_zone.keys()):
            if tid not in active:
                del self._task_zone[tid]
                self._task_tried_zones.pop(tid, None)

    # ------------------------------------------------------------------
    # Episode handling
    # ------------------------------------------------------------------

    def reset_episode(self):
        """Call after RWARE resets the world (also auto-detected in step)."""
        self._task_zone.clear()
        self._task_tried_zones.clear()
        for auction in self.auctions:
            auction.reset_episode()
        self.pathfinder.reset_episode()

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, obs_batch):
        # auto-detect a RWARE reset (step counter went backwards)
        cur = getattr(self.env.unwrapped, "_cur_steps", None)
        if cur is not None:
            if self._last_cur_steps is not None and cur < self._last_cur_steps:
                self.reset_episode()
            self._last_cur_steps = cur

        self._assign_new_tasks()

        all_belief_failed = [self.fault_layers[z].step() for z in range(self.n_zones)]

        failed = list(self.fault_layers[0].actual_failed)
        bids = self.auctions[0].compute_bids(obs_batch)   # once per step

        global_pursue = [False] * self.n_robots
        global_bids = list(bids)
        global_nearest_tasks = [None] * self.n_robots

        for z in range(self.n_zones):
            pursue_z, _, nearest_z = self.auctions[z].step(
                obs_batch,
                belief_failed=all_belief_failed[z],
                bids=bids,
                failed=failed,
            )
            for i in self.auctions[z].zone_robots:
                global_pursue[i] = pursue_z[i]
                global_nearest_tasks[i] = nearest_z[i]

        self._handle_bid_timeouts()
        self._clean_completed_tasks()

        return global_pursue, global_bids, global_nearest_tasks

    # ------------------------------------------------------------------
    # Fault handling
    # ------------------------------------------------------------------

    def get_fault_action_overrides(self, actions):
        """Force NOOP on any robot that has actually failed (single authority)."""
        return self.fault_layers[0].get_actions_override(list(actions))

    def inject_failure(self, robot_id: int):
        ua = self.env.unwrapped
        agent = ua.agents[robot_id]

        # 1. Release shelf if carried so it can be completed by peers
        shelf = agent.carrying_shelf
        if shelf is not None:
            agent.carrying_shelf = None
            shelf_key = self.pathfinder._shelf_key(shelf)
            home = self.pathfinder.shelf_home.get(shelf_key)
            if home is not None:
                shelf.x, shelf.y = home
            else:
                shelf.x, shelf.y = int(agent.x), int(agent.y)
            # Reopen the task for routing & re-auctioning
            tid = (int(shelf.x), int(shelf.y))
            if tid in self._task_zone:
                del self._task_zone[tid]
            if tid in self._task_tried_zones:
                del self._task_tried_zones[tid]
            print(f"[FaultRecovery] Shelf {shelf.id} released from failed Robot {robot_id} -> placed at ({shelf.x}, {shelf.y}) for handoff")

        # 2. Tow the incapacitated robot to the perimeter maintenance bay (Row 0)
        # to clear narrow traffic corridors and delivery goals
        orig_pos = (int(agent.x), int(agent.y))
        agent.x = robot_id % ua.grid_size[1]
        agent.y = 0
        ua._recalc_grid()
        print(f"[FaultRecovery] Robot {robot_id} failed at {orig_pos} -> Towed to maintenance bay ({agent.x}, {agent.y}) to clear traffic aisle")

        # 3. Mark failure in all fault layers
        self.fault_layers[0].inject_failure(robot_id)          # prints once
        for z in range(1, self.n_zones):
            self.fault_layers[z].actual_failed[robot_id] = True  # silent

        # 4. Immediate task handoff: clear all beliefs and claims held by robot_id
        for auction in self.auctions:
            auction.known_claims[robot_id].clear()
            for r in range(self.n_robots):
                auction.known_claims[r] = {
                    tid: c for tid, c in auction.known_claims[r].items() if c != robot_id
                }
                auction.belief[r] = {
                    tid: b for tid, b in auction.belief[r].items() if b[1] != robot_id
                }

    def clear_stale_beliefs(self):
        for auction in self.auctions:
            auction.clear_stale_beliefs()

    # ------------------------------------------------------------------
    # Stats / reporting
    # ------------------------------------------------------------------

    def total_match_count(self) -> int:
        return sum(self.env.match_count)

    def total_mismatch_count(self) -> int:
        return sum(self.env.mismatch_count)

    def avg_comm_per_robot_per_step(self) -> float:
        """Total comm events per step over ALL zones, divided by fleet size."""
        steps = max((len(a.comm_events_per_step) for a in self.auctions), default=0)
        if self.n_robots == 0 or steps == 0:
            return 0.0
        total_per_step = sum(a.avg_comm_per_step() for a in self.auctions)
        return total_per_step / self.n_robots

    def report_stats(self):
        total_deliveries = self.total_match_count() + self.total_mismatch_count()
        match_rate = (
            self.total_match_count() / total_deliveries * 100
            if total_deliveries > 0 else 0.0
        )
        print(f"\n=== HierarchicalCoordinator Stats ===")
        print(f"Total deliveries: {total_deliveries} (match rate: {match_rate:.1f}%)")
        print(f"Total tasks seen: {self.total_tasks_seen}")
        print(f"Cross-zone handoffs: {self.cross_zone_handoffs} "
              f"({self.cross_zone_handoffs / max(self.total_tasks_seen, 1) * 100:.1f}% of tasks)")
        print(f"Tasks routed by zone: {dict(self.tasks_routed_by_zone)}")
        print(f"Avg comm events per robot per step: "
              f"{self.avg_comm_per_robot_per_step():.4f}")
        print()
        for auction in self.auctions:
            auction.report_stats()