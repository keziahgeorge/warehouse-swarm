"""
Phase 7 baseline: greedy nearest-robot allocation.
No bidding, no eligibility check, no communication -- whichever robot is
physically closest to a task pursues it. This is the naive baseline used
to show why capability-aware bidding (Phase 3-5) actually matters.

Reuses the same A* pathfinding from Phase 5 for movement, so the only
difference from your main pipeline is the task-assignment logic itself.
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from pathfinding import GridPathfinder

SEEDS = [1, 42, 100, 2024]
N_STEPS = 20000
ROBOT_TYPES = [0, 1, 2, 0]


def distance(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


def greedy_assign(env):
    """
    For every task in the request queue, assign it to whichever robot is
    physically closest -- no capability check, no communication, no bidding.
    Multiple tasks can be claimed by different robots in the same step;
    if a robot is closest to more than one task, it only pursues its single
    nearest one (ties broken by lowest robot_id, consistent with the rest
    of your project).
    """
    agents = env.unwrapped.agents
    queue = env.unwrapped.request_queue
    n_robots = len(agents)

    nearest_task_per_robot = [None] * n_robots
    pursue = [False] * n_robots

    if not queue:
        return pursue, nearest_task_per_robot

    # each robot's own nearest task, regardless of whether it can match it
    for i, agent in enumerate(agents):
        best_dist, best_shelf = None, None
        for shelf in queue:
            d = distance(agent, shelf)
            if best_dist is None or d < best_dist:
                best_dist, best_shelf = d, shelf
        nearest_task_per_robot[i] = (best_shelf.x, best_shelf.y)

    # for each task, find the single closest robot among those targeting it
    task_best_robot = {}
    for i, agent in enumerate(agents):
        task = nearest_task_per_robot[i]
        d = distance(agent, next(s for s in queue if (s.x, s.y) == task))
        if task not in task_best_robot or d < task_best_robot[task][0] or \
           (d == task_best_robot[task][0] and i < task_best_robot[task][1]):
            task_best_robot[task] = (d, i)

    for task, (d, winner) in task_best_robot.items():
        pursue[winner] = True

    return pursue, nearest_task_per_robot


def run_seed(seed):
    vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=ROBOT_TYPES, seed=seed)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    pathfinder = GridPathfinder(env)

    for step in range(N_STEPS):
        pursue, nearest_tasks = greedy_assign(env)
        actions = pathfinder.get_actions_for_pursuit(obs, pursue, nearest_tasks)
        obs, rewards, dones, infos = vec_env.step(actions)

    total_matches = sum(env.match_count)
    total_mismatches = sum(env.mismatch_count)
    total_deliveries = total_matches + total_mismatches
    match_rate = (total_matches / total_deliveries * 100) if total_deliveries > 0 else 0.0

    vec_env.close()
    return total_deliveries, match_rate


print(f"Running greedy nearest-robot baseline: {len(SEEDS)} seeds, {N_STEPS} steps each.\n")

deliveries_list = []
match_rate_list = []

for seed in SEEDS:
    deliveries, match_rate = run_seed(seed)
    deliveries_list.append(deliveries)
    match_rate_list.append(match_rate)
    print(f"seed={seed}: {deliveries} deliveries, {match_rate:.1f}% match rate")

avg_deliveries = np.mean(deliveries_list)
avg_match_rate = np.mean(match_rate_list)

print(f"\n=== Greedy Baseline Summary ===")
print(f"Average deliveries: {avg_deliveries:.1f} (raw: {deliveries_list})")
print(f"Average match rate: {avg_match_rate:.1f}% (raw: {match_rate_list})")
print(f"\nCompare against your eligibility-gated bidding system: 100.0% match rate, ~14.2 avg deliveries (0-failure baseline)")