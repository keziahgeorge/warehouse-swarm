"""
Phase 7: no-communication ablation experiment. Same failure conditions and
seeds as the Phase 6 degradation curve, but using NoCommAuctionLayer instead
of DecentralizedAuctionLayer -- isolates how much of the fault-tolerance
result depends on communication/reallocation.

Note: failed robots still get forced to NOOP via FaultLayer's action
override (that part is a physical fact of the robot being broken, not a
communication feature) -- what's missing here is other robots' ABILITY
to detect the failure and reallocate the failed robot's claimed tasks.
"""

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer_nocomm import NoCommAuctionLayer
from pathfinding import GridPathfinder
from fault_layer import FaultLayer

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000
FAILURE_STEP = 500
ROBOT_TYPES = [0, 1, 2, 0]

FAILURE_CONDITIONS = {
    "0_failures": [],
    "1_failure": [1],
    "2_failures": [1, 2],
}


def run_condition(failed_robots, seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = NoCommAuctionLayer(env, bid_model_path="bid_head_trained.pt")
    pathfinder = GridPathfinder(env)
    fault_layer = FaultLayer(env)  # still used to force NOOP on failed robots physically

    injected = False

    for step in range(N_STEPS):
        if not injected and step == FAILURE_STEP and failed_robots:
            for r in failed_robots:
                fault_layer.inject_failure(r)
            injected = True

        fault_layer.step()  # updates internal state but beliefs are unused here

        pursue, bids, nearest_tasks = auction.step(obs)  # no belief_failed passed in
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        actions = fault_layer.get_actions_override(actions)  # failed robots physically can't move

        obs, rewards, dones, infos = vec_env.step(actions)

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0

    vec_env.close()
    return total_deliveries, match_rate


print(f"Running no-communication ablation: {len(FAILURE_CONDITIONS)} conditions x {len(SEEDS)} seeds, "
      f"{N_STEPS} steps each.\n")

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

print("\n=== NO-COMMUNICATION ABLATION SUMMARY ===")
baseline_deliveries = results["0_failures"][0]
for condition_name, (avg_deliveries, avg_match_rate, raw_list) in results.items():
    pct_of_baseline = (avg_deliveries / baseline_deliveries * 100) if baseline_deliveries > 0 else 0.0
    print(f"{condition_name}: avg {avg_deliveries:.1f} deliveries "
          f"({pct_of_baseline:.1f}% of no-failure baseline), "
          f"avg match rate {avg_match_rate:.1f}%, raw={raw_list}")

print("\nCompare against your WITH-communication system:")
print("0_failures: 14.2 avg deliveries (100.0%), 100.0% match rate")
print("1_failure: 13.8 avg deliveries (96.5%), 100.0% match rate")
print("2_failures: 10.8 avg deliveries (75.4%), 100.0% match rate")
    