import numpy as np

class RunningNormalizer:
    """Tracks running mean and variance of observations (Welford's algorithm)
    and normalizes new observations using these running statistics."""

    def __init__(self, shape, epsilon=1e-8):
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = epsilon

    def update(self, x):
        x = np.asarray(x, dtype=np.float64)
        batch_mean = x
        batch_var = np.zeros_like(x)
        batch_count = 1

        delta = batch_mean - self.mean
        total_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / total_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + delta ** 2 * self.count * batch_count / total_count
        new_var = m2 / total_count

        self.mean = new_mean
        self.var = new_var
        self.count = total_count

    def normalize(self, x):
        return (np.asarray(x, dtype=np.float64) - self.mean) / (np.sqrt(self.var) + 1e-8)