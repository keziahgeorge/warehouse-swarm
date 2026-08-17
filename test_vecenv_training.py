from stable_baselines3 import PPO
from multi_robot_vecenv import MultiRobotVecEnv

vec_env = MultiRobotVecEnv(seed=42)

model = PPO("MlpPolicy", vec_env, verbose=1, n_steps=256, batch_size=64)

print("Starting a short training run to confirm everything wires up correctly...")
model.learn(total_timesteps=5000)

print("\nTraining completed without errors.")
print("Testing the trained model on one step:")
obs = vec_env.reset()
actions, _ = model.predict(obs)
print("Predicted actions for both robots:", actions)