from stable_baselines3 import PPO
from multi_robot_vecenv import MultiRobotVecEnv
import numpy as np

model = PPO.load("ppo_warehouse_1M")   # <-- changed from ppo_warehouse_500k
vec_env = MultiRobotVecEnv(seed=999)
obs = vec_env.reset()

n_steps = 100000
forward_attempts = [0] * vec_env.n_robots

FORWARD_ACTION = 1

for step in range(n_steps):
    actions, _ = model.predict(obs, deterministic=False)
    for i, a in enumerate(actions):
        if int(a) == FORWARD_ACTION:
            forward_attempts[i] += 1
    obs, rewards, dones, infos = vec_env.step(actions)

env = vec_env.hetero_env
print("\n=== Action Opportunity Diagnostic (1M model) ===")
for i in range(env.unwrapped.n_agents):
    robot_type = env.TYPE_NAMES[env.robot_types[i]]
    matches = env.match_count[i]
    mismatches = env.mismatch_count[i]
    total = matches + mismatches
    match_rate = (matches / total * 100) if total > 0 else 0.0
    print(f"Robot {i} ({robot_type}): {total} total deliveries | "
          f"{matches} matches | {mismatches} mismatches | match rate: {match_rate:.1f}%")
    print(f"  Deliveries by task type: {env.deliveries_by_task_type[i]}")

vec_env.close()