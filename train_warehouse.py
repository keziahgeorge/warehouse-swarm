from stable_baselines3 import PPO
from multi_robot_vecenv import MultiRobotVecEnv
from training_monitor import DeliveryTrackingCallback
import time

robot_types = [0, 1, 2, 0]  # fast_light, heavy_load, balanced, fast_light

vec_env = MultiRobotVecEnv(env_id="rware-tiny-4ag-v2", robot_types=robot_types, seed=42)

model = PPO("MlpPolicy", vec_env, verbose=0, n_steps=512, batch_size=64, learning_rate=3e-4)

callback = DeliveryTrackingCallback(window_size=10000)

TOTAL_TIMESTEPS = 500_000  # start here for the 4-robot case; can extend to 1M later

print(f"Starting 4-robot training for {TOTAL_TIMESTEPS} timesteps...")
print("Watching total reward per 10000-step window -- should trend upward if learning is happening.")

model.learn(total_timesteps=TOTAL_TIMESTEPS, callback=callback)

print("\nTraining complete.")
save_name = f"ppo_warehouse_4robot_500k_{int(time.time())}"
model.save(save_name)
print(f"Model saved as {save_name}.zip")