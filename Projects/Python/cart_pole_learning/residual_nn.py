import numpy as np
import torch
import torch.nn as nn
import casadi as ca


# =====================================================
# 1. Define architecture
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


# =====================================================
# 2. Load trained model
# =====================================================

model = ResidualNet()
model.load_state_dict(torch.load("residual_model.pth", map_location="cuda"))
model.eval()


# =====================================================
# 3. Extract weights (NO transpose)
# =====================================================

weights = []
biases = []

for layer in model.net:
    if isinstance(layer, nn.Linear):
        W = layer.weight.detach().numpy()   # (out, in)
        b = layer.bias.detach().numpy()

        weights.append(W)
        biases.append(b)


# =====================================================
# 4. Load normalization
# =====================================================

input_mean  = ca.DM(np.load("input_mean.npy"))
input_std   = ca.DM(np.load("input_std.npy"))
output_mean = ca.DM(np.load("output_mean.npy"))
output_std  = ca.DM(np.load("output_std.npy"))


# =====================================================
# 5. Build CasADi function (SX version)
# =====================================================

def build_residual_nn():

    x = ca.SX.sym("x", 4)
    u = ca.SX.sym("u", 1)

    xu = ca.vertcat(x, u)

    # Normalize
    z = (xu - input_mean) / input_std

    for i in range(len(weights)):
        W = ca.DM(weights[i])   # (out, in)
        b = ca.DM(biases[i])

        z = W @ z + b

        if i < len(weights) - 1:
            z = ca.tanh(z)

    # Denormalize
    res = z * output_std + output_mean

    # Optional clip (safety)
    res = ca.fmin(ca.fmax(res, -120), 120)

    return ca.Function("residual_nn", [x, u], [res])
