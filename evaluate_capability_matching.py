from stable_baselines3 import PPO
from multi_robot_vecenv import MultiRobotVecEnv

def run_evaluation(model, n_steps=20000, label=""):
    vec_env = MultiRobotVecEnv(seed=999)
    obs = vec_env.reset()

    for step in range(n_steps):
        actions, _ = model.predict(obs, deterministic=False)
        obs, rewards, dones, infos = vec_env.step(actions)

    env = vec_env.hetero_env
    print(f"\n=== Results: {label} ===")
    for i in range(env.unwrapped.n_agents):
        robot_type = env.TYPE_NAMES[env.robot_types[i]]
        matches = env.match_count[i]
        mismatches = env.mismatch_count[i]
        total = matches + mismatches
        match_rate = (matches / total * 100) if total > 0 else 0.0
        print(f"Robot {i} ({robot_type}): {total} total deliveries | "
              f"{matches} matches | {mismatches} mismatches | "
              f"match rate: {match_rate:.1f}%")
        print(f"  Deliveries by task type: {env.deliveries_by_task_type[i]}")

    vec_env.close()


print("Evaluating UNTRAINED (random) policy as baseline...")
untrained_model = PPO("MlpPolicy", MultiRobotVecEnv(seed=1), verbose=0)
run_evaluation(untrained_model, n_steps=100000, label="Untrained (random) policy")

print("\nLoading and evaluating TRAINED policy (500k)...")
trained_model = PPO.load("ppo_warehouse_500k")
run_evaluation(trained_model, n_steps=100000, label="Trained policy (500k timesteps)")