"""
Phase 8: Hierarchical Zone-Based Coordination Experiment

Compares three configurations to demonstrate that per-robot communication
overhead scales with ZONE SIZE (roughly constant) rather than TOTAL FLEET
SIZE (which grows) -- the central scalability claim of the hierarchical
zone extension.

Configurations
--------------
  baseline_flat_8    : 8 robots, 1 zone (flat), rware-tiny-8ag-v2
                       (same as Phase 7 scalability result -- sanity check)
  hierarchical_16_4z : 16 robots, 4 zones, rware-small-16ag-v2 (20x10 grid)
  hierarchical_16_4z_med : 16 robots, 4 zones, rware-medium-16ag-v2 (20x16 grid)

For the baseline (flat, 1 zone), we reuse DecentralizedAuctionLayer directly
with a wrapper that exposes the same comm-per-robot-per-step metric, so the
comparison is apples-to-apples.

Metrics collected per run
-------------------------
  - Total deliveries and match rate (same as Phase 6/7 experiments)
  - Avg comm events per robot per step (KEY scaling metric)
  - Cross-zone handoff rate (sanity check: should be low ~<5%)
  - Per-zone robot count and per-zone delivery breakdown

Seeds and steps: same as prior phases (SEEDS=[1,42,100,2024], N_STEPS=20000)
so results are directly comparable.
"""

import os
import sys
from pathlib import Path

# Ensure core, experiments, and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from collections import defaultdict

from multi_robot_vecenv import MultiRobotVecEnv
from hetero_wrapper import HeterogeneousWarehouseWrapper
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator

# For flat baseline — reuse existing auction + pathfinder
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20_000

# ---------------------------------------------------------------------------
# Configuration table
# Confirmed env IDs (verified against installed RWARE before running):
#   rware-tiny-8ag-v2   -> grid (11,10), 8 agents
#   rware-small-16ag-v2 -> grid (20,10), 16 agents
#   rware-medium-16ag-v2-> grid (20,16), 16 agents
# ---------------------------------------------------------------------------

CONFIGS = {
    "baseline_flat_8": {
        "env_id": "rware-tiny-8ag-v2",
        "robot_types": [0, 1, 2, 0, 1, 2, 0, 1],
        "n_zones": 1,   # flat = single zone = existing system
        "mode": "flat",
        "description": "8 robots, flat (Phase 7 baseline replication)",
    },
    "hierarchical_16_small": {
        "env_id": "rware-small-16ag-v2",
        "robot_types": [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0],
        "n_zones": 4,
        "mode": "hierarchical",
        "description": "16 robots, 4 zones, small grid (20x10)",
    },
    "hierarchical_16_medium": {
        "env_id": "rware-medium-16ag-v2",
        "robot_types": [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0],
        "n_zones": 4,
        "mode": "hierarchical",
        "description": "16 robots, 4 zones, medium grid (20x16)",
    },
}


# ===========================================================================
# Flat baseline runner (reuses existing DecentralizedAuctionLayer verbatim)
# Adds comm-per-robot-per-step measurement for fair comparison.
# ===========================================================================

class _FlatAuctionWithMetrics:
    """
    Thin wrapper around DecentralizedAuctionLayer that counts per-step
    intra-robot communication events (same definition as ZoneAuctionLayer:
    one event = one bid or claim broadcast that reaches a peer within range).
    """

    def __init__(self, hetero_env, bid_model_path="bid_head_trained.pt"):
        self._auction = DecentralizedAuctionLayer(hetero_env, bid_model_path)
        self.comm_events_per_step = []
        self.env = hetero_env
        self.n_robots = hetero_env.unwrapped.n_agents

    def step(self, obs_batch, belief_failed=None):
        pursue, bids, nearest_tasks = self._auction.step(obs_batch, belief_failed)
        # Count how many same-range broadcasts happened this step by
        # re-examining distances (mirrors ZoneAuctionLayer's counter)
        agents = self.env.unwrapped.agents
        events = 0
        for i, agent_i in enumerate(agents):
            for j, agent_j in enumerate(agents):
                if i == j:
                    continue
                dist = abs(agent_i.x - agent_j.x) + abs(agent_i.y - agent_j.y)
                if dist <= DecentralizedAuctionLayer.COMM_RANGE:
                    events += 1
        # events double-counts (i->j and j->i both counted); halve for consistency
        self.comm_events_per_step.append(events // 2)
        return pursue, bids, nearest_tasks

    def clear_stale_beliefs(self):
        self._auction.clear_stale_beliefs()

    def avg_comm_per_robot_per_step(self) -> float:
        if not self.comm_events_per_step:
            return 0.0
        return np.mean(self.comm_events_per_step) / self.n_robots


# ===========================================================================
# Run helpers
# ===========================================================================

def run_flat(env_id, robot_types, seed):
    """Run the flat (existing) system with comm metric instrumentation."""
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = _FlatAuctionWithMetrics(env)
    pathfinder = GridPathfinder(env)

    for step in range(N_STEPS):
        pursue, bids, nearest_tasks = auction.step(obs)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0
    comm_per_robot = auction.avg_comm_per_robot_per_step()
    n_robots = vec_env.n_robots

    vec_env.close()
    return {
        "deliveries": total_deliveries,
        "match_rate": match_rate,
        "comm_per_robot_per_step": comm_per_robot,
        "n_robots": n_robots,
        "cross_zone_handoffs": 0,
        "cross_zone_handoff_rate": 0.0,
    }


def run_hierarchical(env_id, robot_types, n_zones, seed):
    """Run the hierarchical zone system."""
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    # Build zone partitioner
    ua = env.unwrapped
    grid_w, grid_h = ua.grid_size[1], ua.grid_size[0]
    partitioner = ZonePartitioner(grid_w, grid_h, n_zones=n_zones)

    # Assign robots to zones based on start positions
    zone_assignment = partitioner.assign_robots_to_zones(ua.agents)

    # Print zone setup on first seed
    if seed == SEEDS[0]:
        partitioner.print_zone_map()
        partitioner.report_robot_assignment(zone_assignment)

    # Build coordinator
    coordinator = HierarchicalCoordinator(
        env, partitioner, zone_assignment, bid_model_path="bid_head_trained.pt"
    )

    for step in range(N_STEPS):
        pursue, bids, nearest_tasks = coordinator.step(obs)
        actions = coordinator.pathfinder.get_actions_for_pursuit(
            obs, pursue, nearest_tasks
        )
        actions = coordinator.get_fault_action_overrides(actions)
        obs, rewards, dones, infos = vec_env.step(actions)
        coordinator.clear_stale_beliefs()

    total_matches = coordinator.total_match_count()
    total_mismatches = coordinator.total_mismatch_count()
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0
    comm_per_robot = coordinator.avg_comm_per_robot_per_step()
    handoffs = coordinator.cross_zone_handoffs
    tasks_seen = coordinator.total_tasks_seen
    handoff_rate = (handoffs / tasks_seen * 100) if tasks_seen > 0 else 0.0
    n_robots = vec_env.n_robots

    vec_env.close()
    return {
        "deliveries": total_deliveries,
        "match_rate": match_rate,
        "comm_per_robot_per_step": comm_per_robot,
        "n_robots": n_robots,
        "cross_zone_handoffs": handoffs,
        "cross_zone_handoff_rate": handoff_rate,
    }


# ===========================================================================
# Main experiment loop
# ===========================================================================

print("=" * 70)
print("Phase 8: Hierarchical Zone-Based Coordination Experiment")
print(f"Configs: {len(CONFIGS)}  |  Seeds: {SEEDS}  |  Steps: {N_STEPS:,}")
print("=" * 70)
print()

all_results = {}

for config_name, cfg in CONFIGS.items():
    print(f"\n--- {config_name} ---")
    print(f"    {cfg['description']}")
    print(f"    env={cfg['env_id']}, n_zones={cfg['n_zones']}")
    print()

    seed_results = []
    for seed in SEEDS:
        try:
            if cfg["mode"] == "flat":
                r = run_flat(cfg["env_id"], cfg["robot_types"], seed)
            else:
                r = run_hierarchical(
                    cfg["env_id"], cfg["robot_types"], cfg["n_zones"], seed
                )
            seed_results.append(r)
            print(f"  seed={seed}: {r['deliveries']} deliveries, "
                  f"{r['match_rate']:.1f}% match, "
                  f"comm/robot/step={r['comm_per_robot_per_step']:.4f}, "
                  f"handoffs={r['cross_zone_handoffs']} "
                  f"({r['cross_zone_handoff_rate']:.1f}%)")
        except Exception as e:
            print(f"  seed={seed}: FAILED -> {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()

    if not seed_results:
        print(f"  --> {config_name}: no successful runs, skipping")
        continue

    # Aggregate
    avg = {
        key: np.mean([r[key] for r in seed_results])
        for key in ["deliveries", "match_rate", "comm_per_robot_per_step",
                    "cross_zone_handoffs", "cross_zone_handoff_rate"]
    }
    avg["n_robots"] = seed_results[0]["n_robots"]
    avg["per_robot_deliveries"] = avg["deliveries"] / avg["n_robots"]

    all_results[config_name] = avg
    print(f"\n  --> AVERAGE: {avg['deliveries']:.1f} deliveries "
          f"({avg['per_robot_deliveries']:.2f}/robot), "
          f"{avg['match_rate']:.1f}% match, "
          f"comm/robot/step={avg['comm_per_robot_per_step']:.4f}, "
          f"handoff rate={avg['cross_zone_handoff_rate']:.1f}%\n")


# ===========================================================================
# Summary table
# ===========================================================================

print("\n" + "=" * 70)
print("PHASE 8 RESULTS SUMMARY")
print("=" * 70)

header = (f"{'Config':<28} {'Robots':>6} {'Zones':>5} "
          f"{'Deliveries':>10} {'Match%':>7} "
          f"{'Comm/robot/step':>16} {'Handoff%':>9}")
print(header)
print("-" * len(header))

for config_name, r in all_results.items():
    cfg = CONFIGS[config_name]
    print(f"{config_name:<28} {r['n_robots']:>6} {cfg['n_zones']:>5} "
          f"{r['deliveries']:>10.1f} {r['match_rate']:>7.1f} "
          f"{r['comm_per_robot_per_step']:>16.4f} "
          f"{r['cross_zone_handoff_rate']:>9.1f}")

print()
print("KEY SCALING CLAIM:")
if "baseline_flat_8" in all_results and "hierarchical_16_small" in all_results:
    flat_comm = all_results["baseline_flat_8"]["comm_per_robot_per_step"]
    hier_comm = all_results["hierarchical_16_small"]["comm_per_robot_per_step"]
    ratio = hier_comm / flat_comm if flat_comm > 0 else float("nan")
    print(f"  Flat (8 robots)   comm/robot/step: {flat_comm:.4f}")
    print(f"  Zoned (16 robots) comm/robot/step: {hier_comm:.4f}")
    print(f"  Ratio (zoned/flat): {ratio:.2f}x")
    if ratio < 1.5:
        print("  -> Per-robot comm overhead stays roughly CONSTANT "
              "as fleet doubles (zoning works as intended).")
    else:
        print("  -> Per-robot comm overhead increased — review zone sizing.")
else:
    print("  (not enough configs completed to compute scaling ratio)")

print()
print("Note: Match rate should be 100% for all configs (eligibility gate unchanged).")
print("Note: Handoff rate should be low (<10%) if zone sizing is adequate.")
