"""
Collects (observation, label) pairs for training the bid_head.
Label combines:
  - capability match (type vs nearest task's acceptable set)
  - battery sufficiency (enough charge to reach the task, given drain rate)
  - actual delivery outcome (overrides heuristic for the few steps right
    before a real delivery, since real outcome > heuristic estimate)
"""

import os
import sys
from pathlib import Path

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_MODELS_DIR = _ROOT_DIR / "models_and_data"
for _p in [str(_CORE_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from collections import deque
from multi_robot_vecenv import MultiRobotVecEnv

ENV_ID = "rware-tiny-2ag-v2"
ROBOT_TYPES = None
N_STEPS = 200000
OUTPUT_FILE = str(_MODELS_DIR / "bid_training_data.npz")

BUFFER_LEN = 5
BATTERY_SAFETY_MARGIN = 1.2


def collect_data():
    vec_env = MultiRobotVecEnv(env_id=ENV_ID, robot_types=ROBOT_TYPES, seed=123)
    obs = vec_env.reset()
    env = vec_env.hetero_env
    n_robots = vec_env.n_robots

    all_obs = []
    all_labels = []

    # per-robot buffer: stores INDICES into all_labels for the last BUFFER_LEN
    # steps, so we can go back and overwrite them if a real delivery follows
    pending_indices = [deque(maxlen=BUFFER_LEN) for _ in range(n_robots)]

    print(f"Collecting bid-training data over {N_STEPS} steps...")

    for step in range(N_STEPS):
        actions = np.array([vec_env.action_space.sample() for _ in range(n_robots)])

        queue_before = list(env.unwrapped.request_queue)
        shelf_task_before = dict(env.shelf_task_type)

        for i, agent in enumerate(env.unwrapped.agents):
            queue = env.unwrapped.request_queue
            if queue:
                best_dist = None
                best_shelf = None
                for shelf in queue:
                    dist = abs(shelf.x - agent.x) + abs(shelf.y - agent.y)
                    if best_dist is None or dist < best_dist:
                        best_dist = dist
                        best_shelf = shelf

                nearest_task_type = env.shelf_task_type.get(best_shelf, 2)
                robot_type = env.robot_types[i]

                capability_match = 1.0 if robot_type in env.ACCEPTABLE_ROBOTS[nearest_task_type] else 0.0

                drain_rate = env.BATTERY_DRAIN[robot_type]
                battery_needed = best_dist * drain_rate * BATTERY_SAFETY_MARGIN
                battery_sufficient = 1.0 if env.battery[i] >= battery_needed else 0.0

                heuristic_label = 0.6 * capability_match + 0.4 * battery_sufficient

                # save immediately, every step
                all_obs.append(obs[i].copy())
                all_labels.append(heuristic_label)
                pending_indices[i].append(len(all_labels) - 1)

        obs, rewards, dones, infos = vec_env.step(actions)

        queue_after = set(env.unwrapped.request_queue)
        delivered_shelves = [s for s in queue_before if s not in queue_after]

        for shelf in delivered_shelves:
            task_type = shelf_task_before.get(shelf, 2)
            for i, agent in enumerate(env.unwrapped.agents):
                if agent.x == shelf.x and agent.y == shelf.y:
                    real_outcome = 1.0 if env.robot_types[i] in env.ACCEPTABLE_ROBOTS[task_type] else 0.0
                    # overwrite the labels of the last few steps for this robot
                    for idx in pending_indices[i]:
                        all_labels[idx] = real_outcome
                    pending_indices[i].clear()

        if step % 20000 == 0:
            print(f"  step {step}, samples so far: {len(all_obs)}")

    all_obs = np.array(all_obs, dtype=np.float32)
    all_labels = np.array(all_labels, dtype=np.float32)

    print(f"\nDone. Total samples: {len(all_obs)}")
    print(f"Mean label value: {all_labels.mean():.3f}")
    print(f"Label std dev: {all_labels.std():.3f}")

    np.savez(OUTPUT_FILE, obs=all_obs, labels=all_labels)
    print(f"Saved to {OUTPUT_FILE}")

    vec_env.close()


if __name__ == "__main__":
    collect_data()