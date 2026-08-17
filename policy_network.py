import torch
import torch.nn as nn

class CapabilityConditionedPolicy(nn.Module):
    """
    Shared policy network used by all robots (IPPO-style).
    Takes the 80-dim heterogeneous observation and outputs:
      - bid_value: scalar in [0, 1], how suitable this robot is for the nearest task
      - comm_gate: scalar in [0, 1] (probability of broadcasting this step)

    Because the observation includes the robot's own type embedding,
    the same shared weights naturally produce different outputs for
    different robot types.
    """

    def __init__(self, obs_dim=80, hidden1=128, hidden2=64):
        super().__init__()

        self.encoder = nn.Sequential(
            nn.Linear(obs_dim, hidden1),
            nn.ReLU(),
            nn.Linear(hidden1, hidden2),
            nn.ReLU(),
        )

        self.bid_head = nn.Linear(hidden2, 1)
        self.comm_head = nn.Linear(hidden2, 1)

    def forward(self, obs):
        """
        obs: tensor of shape (batch_size, 80)
        returns: bid_value (batch_size, 1), comm_gate (batch_size, 1)
                 both squashed into [0, 1] via sigmoid
        """
        features = self.encoder(obs)
        bid_value = torch.sigmoid(self.bid_head(features))
        comm_gate = torch.sigmoid(self.comm_head(features))
        return bid_value, comm_gate