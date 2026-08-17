import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, n_actions, hidden=64):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
        )
        self.actor_head = nn.Linear(hidden, n_actions)
        self.critic_head = nn.Linear(hidden, 1)

    def forward(self, obs):
        features = self.shared(obs)
        action_logits = self.actor_head(features)
        value = self.critic_head(features)
        return action_logits, value

    def get_action(self, obs):
        action_logits, value = self.forward(obs)
        dist = torch.distributions.Categorical(logits=action_logits)
        action = dist.sample()
        log_prob = dist.log_prob(action)
        return action.item(), log_prob.item(), value.item()

    def evaluate_actions(self, obs, actions):
        action_logits, values = self.forward(obs)
        dist = torch.distributions.Categorical(logits=action_logits)
        log_probs = dist.log_prob(actions)
        entropy = dist.entropy()
        return log_probs, entropy, values.squeeze(-1)


def compute_returns_and_advantages(rewards, values, gamma=0.99, lam=0.95):
    returns = []
    advantages = []
    gae = 0.0
    next_value = 0.0

    for t in reversed(range(len(rewards))):
        delta = rewards[t] + gamma * next_value - values[t]
        gae = delta + gamma * lam * gae
        advantages.insert(0, gae)
        next_value = values[t]

    for t in range(len(rewards)):
        returns.append(advantages[t] + values[t])

    return returns, advantages


def ppo_update(model, optimizer, obs_batch, action_batch, old_log_probs,
               returns, advantages, clip_eps=0.2, epochs=4, max_grad_norm=0.5, verbose=False):

    obs_batch = torch.tensor(np.array(obs_batch), dtype=torch.float32)
    action_batch = torch.tensor(action_batch, dtype=torch.int64)
    old_log_probs = torch.tensor(old_log_probs, dtype=torch.float32)
    returns = torch.tensor(returns, dtype=torch.float32)
    advantages_raw = torch.tensor(advantages, dtype=torch.float32)

    advantages = (advantages_raw - advantages_raw.mean()) / (advantages_raw.std() + 1e-8)

    for epoch in range(epochs):
        new_log_probs, entropy, values = model.evaluate_actions(obs_batch, action_batch)

        ratio = torch.exp(new_log_probs - old_log_probs)
        clipped_ratio = torch.clamp(ratio, 1 - clip_eps, 1 + clip_eps)

        policy_loss = -torch.min(ratio * advantages, clipped_ratio * advantages).mean()
        value_loss = F.mse_loss(values, returns)
        entropy_bonus = entropy.mean()

        loss = policy_loss + 0.5 * value_loss - 0.001 * entropy_bonus

        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)
        optimizer.step()

        if verbose:
            print(f"    [epoch {epoch}] policy_loss={policy_loss.item():.6f} "
                  f"value_loss={value_loss.item():.4f} "
                  f"ratio_mean={ratio.mean().item():.6f}")