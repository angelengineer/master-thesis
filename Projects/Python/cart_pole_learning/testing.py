# En Python puro, compara predicción vs residual real
from residual_nn import build_residual_nn
import casadi as ca
import numpy as np

nn = build_residual_nn()
x_test = ca.DM([0.0, np.pi, 0.0, 0.0])
u_test = ca.DM([10.0])
pred = nn(x_test, u_test)
print("Predicción residual:", pred)
# ¿Es razonable? ¿O da NaN/valores enormes?