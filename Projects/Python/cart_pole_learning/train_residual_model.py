import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split

# =====================================================
# 0. Device (GPU if available)
# =====================================================

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))

# =====================================================
# 1. Load dataset
# =====================================================

X = np.load("X.npy")              # (N, 4)
U = np.load("U.npy")              # (N, 1)
delta_f = np.load("delta_f.npy")  # (N, 4)

# Only acceleration residuals (dv/dt, domega/dt)
delta_acc = delta_f[:, 2:4]       # (N, 2)

# Input = [x, u]
inputs = np.hstack([X, U])        # (N, 5)

print("Dataset size:", inputs.shape[0])

# =====================================================
# 2. Normalization
# =====================================================

input_mean = inputs.mean(axis=0)
input_std  = inputs.std(axis=0) + 1e-8

output_mean = delta_acc.mean(axis=0)
output_std  = delta_acc.std(axis=0) + 1e-8

inputs_norm  = (inputs - input_mean) / input_std
outputs_norm = (delta_acc - output_mean) / output_std

# Save normalization (needed later in NMPC)
np.save("input_mean.npy", input_mean)
np.save("input_std.npy", input_std)
np.save("output_mean.npy", output_mean)
np.save("output_std.npy", output_std)

# =====================================================
# 3. Torch dataset
# =====================================================

inputs_t  = torch.tensor(inputs_norm, dtype=torch.float32)
outputs_t = torch.tensor(outputs_norm, dtype=torch.float32)

dataset = TensorDataset(inputs_t, outputs_t)

train_size = int(0.9 * len(dataset))
val_size = len(dataset) - train_size

train_set, val_set = random_split(dataset, [train_size, val_size])

batch_size = 65536  # Increase if GPU allows

train_loader = DataLoader(
    train_set,
    batch_size=batch_size,
    shuffle=True,
    num_workers=4,
    pin_memory=True
)

val_loader = DataLoader(
    val_set,
    batch_size=batch_size,
    num_workers=4,
    pin_memory=True
)

# =====================================================
# 4. Model
# =====================================================

class ResidualNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(5, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 32),
            nn.Tanh(),
            nn.Linear(32, 2)
        )


    def forward(self, x):
        return self.net(x)

model = ResidualNet().to(device)

# =====================================================
# 5. Training setup
# =====================================================

optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
criterion  = nn.HuberLoss(delta=1.0)


epochs = 30

# =====================================================
# 6. Training loop
# =====================================================

for epoch in range(epochs):

    model.train()
    train_loss = 0.0

    for xb, yb in train_loader:
        xb = xb.to(device, non_blocking=True)
        yb = yb.to(device, non_blocking=True)

        optimizer.zero_grad()
        pred = model(xb)
        loss = criterion(pred, yb)
        loss.backward()
        optimizer.step()

        train_loss += loss.item()

    train_loss /= len(train_loader)

    model.eval()
    val_loss = 0.0

    with torch.no_grad():
        for xb, yb in val_loader:
            xb = xb.to(device, non_blocking=True)
            yb = yb.to(device, non_blocking=True)

            pred = model(xb)
            loss = criterion(pred, yb)
            val_loss += loss.item()

    val_loss /= len(val_loader)

    print(f"Epoch {epoch+1:02d} | Train: {train_loss:.6f} | Val: {val_loss:.6f}")

# =====================================================
# 7. Save model
# =====================================================

torch.save(model.state_dict(), "residual_model.pth")

print("✓ Model saved successfully")

# =====================================================
# 8. Evaluate real physical error
# =====================================================

model.eval()
for p in model.parameters():
    p.requires_grad = False
    
torch.save(model.state_dict(), "residual_model.pt")

with torch.no_grad():
    inputs_full = torch.tensor(inputs_norm, dtype=torch.float32).to(device)
    pred_norm = model(inputs_full).cpu().numpy()

# Denormalize
pred_real = pred_norm * output_std + output_mean

mse_real = np.mean((pred_real - delta_acc)**2)
rmse_real = np.sqrt(mse_real)

print("Real RMSE (accelerations):", rmse_real)

mask = np.linalg.norm(delta_acc, axis=1) < 50
mse_small = np.mean((pred_real[mask] - delta_acc[mask])**2)
rmse_small = np.sqrt(mse_small)

print("RMSE small region:", rmse_small)

# Diagnóstico rápido (ejecuta esto):
print("Rango real de Δẍ:", delta_acc[:,0].min(), delta_acc[:,0].max())
print("Rango real de Δθ̈:", delta_acc[:,1].min(), delta_acc[:,1].max())
print("Media absoluta Δẍ:", np.mean(np.abs(delta_acc[:,0])))
print("Media absoluta Δθ̈:", np.mean(np.abs(delta_acc[:,1])))