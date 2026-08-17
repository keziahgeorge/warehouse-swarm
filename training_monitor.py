from stable_baselines3.common.callbacks import BaseCallback
import numpy as np

class DeliveryTrackingCallback(BaseCallback):
    """Tracks total reward earned over rolling windows of timesteps,
    as a proxy for 'are robots getting better at delivering shelves'."""

    def __init__(self, window_size=2000, verbose=0):
        super().__init__(verbose)
        self.window_size = window_size
        self.rewards_in_window = []
        self.window_count = 0

    def _on_step(self) -> bool:
        # self.locals['rewards'] is the array of rewards from the last step,
        # one per parallel "environment" (i.e., one per robot here)
        rewards = self.locals.get("rewards", None)
        if rewards is not None:
            self.rewards_in_window.append(np.sum(rewards))

        if len(self.rewards_in_window) >= self.window_size:
            self.window_count += 1
            total_reward = sum(self.rewards_in_window)
            avg_reward_per_step = total_reward / len(self.rewards_in_window)
            print(f"[Monitor] Window {self.window_count} | "
                  f"Total timesteps so far: {self.num_timesteps} | "
                  f"Total reward this window: {total_reward:.2f} | "
                  f"Avg reward/step: {avg_reward_per_step:.4f}", flush=True)
            self.rewards_in_window = []

        return True