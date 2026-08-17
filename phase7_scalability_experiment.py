"""
Phase 7: scalability experiment. Runs the full pipeline (bidding +
eligibility-gated auction + A* pathfinding, no failures) at 2, 4, and 8
robots, to see how per-robot and total throughput change as swarm size
increases. Uses RWARE's built-in 8-agent config.
"""

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from auction_layer import DecentralizedAuctionLayer
from pathfinding import GridPathfinder

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000

# robot type assignments per swarm size (cycling fast_light/heavy_load/balanced)
SWARM_CONFIGS = {
    "2_robots": ("rware-tiny-2ag-v2", [0, 1]),
    "4_robots": ("rware-tiny-4ag-v2", [0, 1, 2, 0]),
    "8_robots": ("rware-tiny-8ag-v2", [0, 1, 2, 0, 1, 2, 0, 1]),
}


def run_seed(env_id, robot_types, seed):
    vec_env = MultiRobotVecEnv(env_id=env_id, robot_types=robot_types, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env

    auction = DecentralizedAuctionLayer(env, bid_model_path="bid_head_trained.pt")
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
    n_robots = vec_env.n_robots

    vec_env.close()
    return total_deliveries, match_rate, n_robots


print(f"Running scalability experiment: {len(SWARM_CONFIGS)} swarm sizes x {len(SEEDS)} seeds, "
      f"{N_STEPS} steps each. This will take a while.\n")

results = {}

for config_name, (env_id, robot_types) in SWARM_CONFIGS.items():
    deliveries_list = []
    match_rate_list = []
    n_robots = None

    for seed in SEEDS:
        try:
            deliveries, match_rate, n_robots = run_seed(env_id, robot_types, seed)
        except Exception as e:
            print(f"  [{config_name}] seed={seed}: FAILED with error: {e}")
            continue

        deliveries_list.append(deliveries)
        match_rate_list.append(match_rate)
        print(f"  [{config_name}] seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

    if not deliveries_list:
        print(f"  --> {config_name}: no successful runs, skipping\n")
        continue

    avg_deliveries = np.mean(deliveries_list)
    avg_match_rate = np.mean(match_rate_list)
    per_robot_deliveries = avg_deliveries / n_robots
    results[config_name] = (avg_deliveries, avg_match_rate, per_robot_deliveries, n_robots)
    print(f"  --> {config_name} AVERAGE: {avg_deliveries:.1f} deliveries "
          f"({per_robot_deliveries:.2f} per robot), {avg_match_rate:.1f}% match rate\n")

print("\n=== SCALABILITY SUMMARY ===")
for config_name, (avg_deliveries, avg_match_rate, per_robot, n_robots) in results.items():
    print(f"{config_name} ({n_robots} robots): {avg_deliveries:.1f} total deliveries, "
          f"{per_robot:.2f} per-robot deliveries, {avg_match_rate:.1f}% match rate")