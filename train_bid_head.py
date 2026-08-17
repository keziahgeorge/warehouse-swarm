"""
Trains only the bid_head of CapabilityConditionedPolicy using supervised
regression against a continuous suitability label (capability + battery +
real delivery outcome blend). Reports MAE instead of thresholded accuracy,
since labels are continuous, not binary.
"""

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader
from policy_network import CapabilityConditionedPolicy

DATA_FILE = "bid_training_data.npz"
MODEL_OUT = "bid_head_trained.pt"

EPOCHS = 60
BATCH_SIZE = 256
LEARNING_RATE = 3e-3
VAL_SPLIT = 0.15

data = np.load(DATA_FILE)
obs = torch.tensor(data["obs"], dtype=torch.float32)
labels = torch.tensor(data["labels"], dtype=torch.float32).unsqueeze(1)

n_total = len(obs)
n_val = int(n_total * VAL_SPLIT)
n_train = n_total - n_val

perm = torch.randperm(n_total)
train_idx, val_idx = perm[:n_train], perm[n_train:]

train_ds = TensorDataset(obs[train_idx], labels[train_idx])
val_ds = TensorDataset(obs[val_idx], labels[val_idx])

train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False)

model = CapabilityConditionedPolicy(obs_dim=80)
optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.3)
bce_loss_fn = nn.BCELoss()
mae_loss_fn = nn.L1Loss()

print(f"Training on {n_train} samples, validating on {n_val} samples")

for epoch in range(EPOCHS):
    model.train()
    train_losses = []
    for batch_obs, batch_labels in train_loader:
        bid_value, _ = model(batch_obs)
        loss = bce_loss_fn(bid_value, batch_labels)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        train_losses.append(loss.item())

    scheduler.step()

    model.eval()
    val_bce_losses = []
    val_mae_losses = []
    with torch.no_grad():
        for batch_obs, batch_labels in val_loader:
            bid_value, _ = model(batch_obs)
            val_bce_losses.append(bce_loss_fn(bid_value, batch_labels).item())
            val_mae_losses.append(mae_loss_fn(bid_value, batch_labels).item())

    train_loss = np.mean(train_losses)
    val_bce = np.mean(val_bce_losses)
    val_mae = np.mean(val_mae_losses)

    print(f"Epoch {epoch+1}/{EPOCHS} | train_bce: {train_loss:.4f} | "
          f"val_bce: {val_bce:.4f} | val_mae: {val_mae:.4f} | lr: {scheduler.get_last_lr()[0]:.5f}")

torch.save(model.state_dict(), MODEL_OUT)
print(f"\nTraining complete. Saved bid_head weights to {MODEL_OUT}")
print(f"Final val_mae: {val_mae:.4f} (lower is better; 0 = perfect, 0.25 ≈ predicting the mean)")