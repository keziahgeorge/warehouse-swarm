import numpy as np
from stable_baselines3.common.vec_env import VecEnv
from stable_baselines3.common.vec_env.base_vec_env import VecEnvIndices
import gymnasium as gym
import rware
from hetero_wrapper import HeterogeneousWarehouseWrapper


class MultiRobotVecEnv(VecEnv):
    """
    Presents a single HeterogeneousWarehouseWrapper's N robots as N parallel
    'environments' to SB3. Internally, all robots are stepped together in the
    same real environment every call -- this correctly implements IPPO with
    parameter sharing (one SB3 model, N agents, one shared world).
    """

    def __init__(self, env_id="rware-tiny-2ag-v2", robot_types=None, seed=None):
        base_env = gym.make(env_id)
        self.hetero_env = HeterogeneousWarehouseWrapper(base_env, robot_types=robot_types, seed=seed)
        self.n_robots = self.hetero_env.unwrapped.n_agents

        obs_dim = 80  # from our confirmed observation size
        observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        action_space = gym.spaces.Discrete(5)  # single robot's action space

        super().__init__(self.n_robots, observation_space, action_space)

        self._actions = None
        self._last_obs = None

    def reset(self):
        obs_tuple, info = self.hetero_env.reset()
        self._last_obs = np.stack(obs_tuple).astype(np.float32)
        return self._last_obs

    def step_async(self, actions):
        self._actions = actions

    def step_wait(self):
        # SB3 gives us N actions (one per robot); RWARE expects a tuple
        actions_tuple = tuple(int(a) for a in self._actions)

        obs_tuple, rewards, terminated, truncated, info = self.hetero_env.step(actions_tuple)

        obs = np.stack(obs_tuple).astype(np.float32)
        rewards = np.array(rewards, dtype=np.float32)
        dones = np.array([terminated or truncated] * self.n_robots, dtype=bool)
        infos = [{} for _ in range(self.n_robots)]

        # if the episode ended, SB3 expects an auto-reset, matching normal VecEnv behaviour
        if terminated or truncated:
            reset_obs, _ = self.hetero_env.reset()
            obs = np.stack(reset_obs).astype(np.float32)

        self._last_obs = obs
        return obs, rewards, dones, infos

    def close(self):
        self.hetero_env.close()

    # --- required abstract methods from VecEnv, minimal implementations ---
    def get_attr(self, attr_name, indices=None):
        return [getattr(self.hetero_env, attr_name)] * self.n_robots

    def set_attr(self, attr_name, value, indices=None):
        setattr(self.hetero_env, attr_name, value)

    def env_method(self, method_name, *args, indices=None, **kwargs):
        method = getattr(self.hetero_env, method_name)
        return [method(*args, **kwargs)] * self.n_robots

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.n_robots

    def seed(self, seed=None):
        return [seed] * self.n_robots