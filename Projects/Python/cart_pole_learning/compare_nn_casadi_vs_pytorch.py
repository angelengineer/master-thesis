import numpy as np
import torch
import torch.nn as nn
import casadi as ca

from residual_nn import build_residual_nn   # tu versión CasADi

# ============================================================
# 1. Define PyTorch model (misma arquitectura)
# ============================================================

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


# ============================================================
# 2. Load trained model
# ============================================================

torch_model = ResidualNet()
torch_model.load_state_dict(torch.load("residual_model.pth", map_location="cpu"))
torch_model.eval()

# Load normalization
input_mean  = np.load("input_mean.npy")
input_std   = np.load("input_std.npy")
output_mean = np.load("output_mean.npy")
output_std  = np.load("output_std.npy")

# ============================================================
# 3. Build CasADi model
# ============================================================

casadi_residual = build_residual_nn()

# ============================================================
# 4. Comparison loop
# ============================================================

n_tests = 1000

max_error = 0.0
mean_error = 0.0

for _ in range(n_tests):

    # random physically reasonable state
    x = np.random.uniform(
        low=[-2, np.pi-1.0, -3, -5],
        high=[ 2, np.pi+1.0,  3,  5]
    )

    u = np.random.uniform(low=[-50], high=[50])

    xu = np.concatenate([x, u])

    # ----- PyTorch forward -----
    xu_norm = (xu - input_mean) / input_std
    xu_tensor = torch.tensor(xu_norm, dtype=torch.float32)

    with torch.no_grad():
        torch_out = torch_model(xu_tensor).numpy()

    torch_out = torch_out * output_std + output_mean

    # ----- CasADi forward -----
    casadi_out = casadi_residual(x, u).full().flatten()

    # ----- Error -----
    err = np.linalg.norm(torch_out - casadi_out)

    max_error = max(max_error, err)
    mean_error += err

mean_error /= n_tests

print("=====================================")
print(f"Tests: {n_tests}")
print(f"Max error: {max_error}")
print(f"Mean error: {mean_error}")
print("=====================================")

if max_error < 1e-6:
    print("✓ PERFECT MATCH (<1e-6)")
elif max_error < 1e-4:
    print("✓ Small numerical difference (<1e-4) — OK")
else:
    print("⚠ WARNING: mismatch too large")
