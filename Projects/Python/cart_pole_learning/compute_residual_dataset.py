import casadi as ca
import numpy as np
from cart_pole_model import cart_pole_model
import matplotlib.pyplot as plt

# =============================
# 1. Cargar dataset
# =============================
dataset = np.load("cartpole_residual_dataset.npy", allow_pickle=True)
print("Dataset size:", len(dataset))

# usar dt del dataset (todos tienen el mismo)
dt = dataset[0]["dt"]

# =============================
# 2. Modelo nominal
# =============================
model = cart_pole_model()

x = model.x
u = model.u
f_expl = model.f_expl_expr

f_nom = ca.Function("f_nom", [x, u], [f_expl])

# =============================
# 3. Convertir a arrays
# =============================
X = []
U = []
X_next = []

for sample in dataset:
    X.append(sample["state"])
    U.append([sample["control"]])
    X_next.append(sample["next_state"])

X = np.array(X)
U = np.array(U)
X_next = np.array(X_next)

# =============================
# 4. Derivada real
# =============================
xdot_real = (X_next - X) / dt

# =============================
# 5. Derivada nominal
# =============================
xdot_nom = np.zeros_like(xdot_real)

for i in range(len(X)):
    xdot_nom[i] = np.array(f_nom(X[i], U[i])).flatten()

# =============================
# 6. Residual
# =============================
delta_f = xdot_real - xdot_nom
magnitudes = np.linalg.norm(delta_f, axis=1)

print("Mean residual magnitude:",
      np.mean(magnitudes))
print("Max residual magnitude:",
      np.max(magnitudes))

print("\nPercentiles:")
for p in [50, 75, 90, 95, 99, 99.9]:
    print(f"{p}%:", np.percentile(magnitudes, p))

plt.figure()
plt.hist(magnitudes, bins=200)
plt.yscale("log")
plt.title("Residual magnitude distribution")
plt.xlabel("||Δf||")
plt.ylabel("Count (log scale)")
plt.show()


threshold = np.percentile(magnitudes, 99.5)

mask = magnitudes <= threshold

X = X[mask]
U = U[mask]
delta_f = delta_f[mask]

print("After filtering:")
print("Samples remaining:", len(X))
print("New max residual:", np.max(np.linalg.norm(delta_f, axis=1)))

# Guardar limpio para entrenamiento
np.save("X.npy", X)
np.save("U.npy", U)
np.save("delta_f.npy", delta_f)




