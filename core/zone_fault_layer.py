"""
Phase 8: Zone-Scoped Fault Layer

A thin wrapper around FaultLayer that restricts heartbeat monitoring to
robots within the same zone. The only change from the base FaultLayer is
that the inner loop in step() skips any robot that lives in a different zone
-- consistent with the hierarchical design where liveness-checking is a
local, zone-internal concern.

All other behaviour is identical: HEARTBEAT_TIMEOUT, belief_failed matrix
structure, inject_failure(), get_actions_override(), and
detection_accuracy_report() are all unchanged. This means the rest of the
pipeline (auction layer, pathfinder) can consume the output of this class
interchangeably with FaultLayer.
"""

from fault_layer import FaultLayer


class ZoneFaultLayer(FaultLayer):
    """
    Heartbeat-based failure detection scoped to intra-zone robots only.

    Parameters
    ----------
    hetero_env      : HeterogeneousWarehouseWrapper
    zone_assignment : dict {robot_id -> zone_id}  (from ZonePartitioner)
    """

    # Use a tighter comm range for heartbeats within a zone — since we no
    # longer need to span the whole warehouse, use the same range as the
    # auction layer (8 cells). Robots in the same zone will always be within
    # this range given the grid sizes we test (tiny=11x10, small=20x10).
    # Override to 20 if you observe false positives (same logic that fixed
    # the original FaultLayer).
    COMM_RANGE = 20  # kept generous; zone filter is the primary restriction

    def __init__(self, hetero_env, zone_assignment: dict):
        super().__init__(hetero_env)
        self.zone_assignment = zone_assignment  # robot_id -> zone_id

    def step(self):
        """
        Identical to FaultLayer.step() except the heartbeat broadcast loop
        additionally skips robot pairs that belong to different zones.

        Returns: belief_failed (n_robots x n_robots bool matrix)
        """
        agents = self.env.unwrapped.agents

        # Every robot's "steps since heard" ticks up by 1
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i != j:
                    self.steps_since_heard[i][j] += 1

        # Active robots broadcast heartbeat — only to same-zone robots within range
        for i, agent_i in enumerate(agents):
            if self.actual_failed[i]:
                continue  # failed robots don't broadcast

            for j, agent_j in enumerate(agents):
                if i == j:
                    continue

                # === Zone filter: only monitor robots in the same zone ===
                if self.zone_assignment.get(i) != self.zone_assignment.get(j):
                    # Different zone — never update the counter; these robots
                    # should not be tracking each other's liveness at all.
                    # Reset to 0 so they never trip the timeout for each other.
                    self.steps_since_heard[j][i] = 0
                    continue

                if self._distance(agent_i, agent_j) <= self.COMM_RANGE:
                    self.steps_since_heard[j][i] = 0

        # Update belief based on timeout
        for i in range(self.n_robots):
            for j in range(self.n_robots):
                if i == j:
                    continue
                # Cross-zone robots are never considered "failed" by each other
                if self.zone_assignment.get(i) != self.zone_assignment.get(j):
                    self.belief_failed[i][j] = False
                    continue
                self.belief_failed[i][j] = (
                    self.steps_since_heard[i][j] > self.HEARTBEAT_TIMEOUT
                )

        return self.belief_failed
