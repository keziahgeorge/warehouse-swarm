"""
Phase 6 headline experiment: fault-tolerance degradation curve.
Runs the full pipeline (bidding + eligibility-gated auction + pathfinding +
heartbeat fault detection + stale-claim reallocation) across different
numbers of simultaneous robot failures (0, 1, 2 out of 4), each repeated
over multiple seeds, with long enough runs to get trustworthy delivery
counts (matching the scale where prior tests showed real signal).

Failures are injected early (step 500) so most of the run happens
post-failure, giving a fair read on sustained degraded throughput.
"""

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]

# which robots fail under each condition (indices into the 4-robot list)
FAILURE_CONDITIONS = {
    "0_failures": [],
    "1_failure": [1],       # heavy_load fails
    "2_failures": [1, 2],   # heavy_load + balanced fail
}


def run_condition(failed_robots, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault_layer = FaultLayer(env)

    injected = False

    for step in range(N_STEPS):
        if not injected and step == FAILURE_STEP and failed_robots:
            for r in failed_robots:
                fault_layer.inject_failure(r)
            injected = True

        belief_failed = fault_layer.step()
        pursue, bids, nearest_tasks = auction.step(obs, belief_failed=belief_failed)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        actions = fault_layer.get_actions_override(actions)

        obs, rewards, dones, infos = vec_env.step(actions)
        auction.clear_stale_beliefs()

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0

    vec_env.close()
    return total_deliveries, match_rate


print(f"Running degradation experiment: {len(FAILURE_CONDITIONS)} conditions x {len(SEEDS)} seeds, "
      f"{N_STEPS} steps each. This will take a while.\n")

results = {}

for condition_name, failed_robots in FAILURE_CONDITIONS.items():
    deliveries_list = []
    match_rate_list = []

    for seed in SEEDS:
        deliveries, match_rate = run_condition(failed_robots, seed)
        deliveries_list.append(deliveries)
        match_rate_list.append(match_rate)
        print(f"  [{condition_name}] seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

    avg_deliveries = np.mean(deliveries_list)
    avg_match_rate = np.mean(match_rate_list)
    results[condition_name] = (avg_deliveries, avg_match_rate, deliveries_list)
    print(f"  --> {condition_name} AVERAGE: {avg_deliveries:.1f} deliveries, {avg_match_rate:.1f}% match rate\n")

print("\n=== DEGRADATION CURVE SUMMARY ===")
baseline_deliveries = results["0_failures"][0]
for condition_name, (avg_deliveries, avg_match_rate, raw_list) in results.items():
    pct_of_baseline = (avg_deliveries / baseline_deliveries * 100) if baseline_deliveries > 0 else 0.0
    print(f"{condition_name}: avg {avg_deliveries:.1f} deliveries "
          f"({pct_of_baseline:.1f}% of no-failure baseline), "
          f"avg match rate {avg_match_rate:.1f}%, raw={raw_list}")