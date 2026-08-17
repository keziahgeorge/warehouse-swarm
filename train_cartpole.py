import gymnasium as gym
import torch
import numpy as np
from simple_ppo import ActorCritic, compute_returns_and_advantages, ppo_update
from running_normalizer import RunningNormalizer

env = gym.make("CartPole-v1")
obs_dim = env.observation_space.shape[0]
n_actions = env.action_space.n

model = ActorCritic(obs_dim, n_actions)
optimizer = torch.optim.Adam(model.parameters(), lr=3e-4)
normalizer = RunningNormalizer(obs_dim)

N_ITERATIONS = 60
MIN_TIMESTEPS_PER_UPDATE = 2000

history = []

for iteration in range(N_ITERATIONS):
    all_obs, all_actions, all_log_probs, all_returns, all_advantages = [], [], [], [], []
    episode_lengths = []
    total_steps = 0

    while total_steps < MIN_TIMESTEPS_PER_UPDATE:
        obs_list, action_list, log_prob_list, value_list, reward_list = [], [], [], [], []

        obs, info = env.reset()
        done = False
        while not done:
            normalizer.update(obs)
            norm_obs = normalizer.normalize(obs)

            obs_tensor = torch.tensor(norm_obs, dtype=torch.float32).unsqueeze(0)
            action, log_prob, value = model.get_action(obs_tensor)

            next_obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            obs_list.append(norm_obs)
            action_list.append(action)
            log_prob_list.append(log_prob)
            value_list.append(value)
            reward_list.append(reward)

            obs = next_obs

        returns, advantages = compute_returns_and_advantages(reward_list, value_list)

        all_obs.extend(obs_list)
        all_actions.extend(action_list)
        all_log_probs.extend(log_prob_list)
        all_returns.extend(returns)
        all_advantages.extend(advantages)
        episode_lengths.append(len(reward_list))
        total_steps += len(reward_list)

    avg_length = sum(episode_lengths) / len(episode_lengths)
    history.append(avg_length)
    running_avg = sum(history[-5:]) / len(history[-5:])
    print(f"Iteration {iteration:3d} | Episodes: {len(episode_lengths):3d} | "
          f"Batch avg len: {avg_length:6.1f} | Running avg (last 5): {running_avg:6.1f}", flush=True)

    ppo_update(model, optimizer, all_obs, all_actions, all_log_probs, all_returns, all_advantages, verbose=False)

env.close()