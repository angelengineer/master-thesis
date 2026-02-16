import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from cart_pole_model import cart_pole_model
# ------------------------------------------------------------
# 1. Modelo dinámico (debes tener esta función en el mismo archivo)
#    Si está en otro módulo, importala: from cart_pole_model import cart_pole_model
# ------------------------------------------------------------


# ============ CONFIGURACIÓN INICIAL ============
print("Configurando OCP para péndulo sobre carro...")

N = 40               # pasos de discretización
T = 1.0              # horizonte [s]
x0 = np.array([0.0, np.pi, 0.0, 0.0])   # péndulo colgando

# ============ MODELO ============
model = cart_pole_model()
nx = model.x.size1()      # 4
nu = model.u.size1()      # 1

# ============ OCP ============
ocp = AcadosOcp()
ocp.model = model

# ============ COSTOS NONLINEAR_LS ============
W_x = np.diag([1e2, 1e3, 1e-2, 1e-2])   # pesos estados
W_u = np.array([[1e-2]])                 # peso control (1x1)

# --- Costo inicial (stage 0) ---
ocp.cost.cost_type_0 = 'NONLINEAR_LS'
ocp.cost.W_0 = W_u
ocp.cost.yref_0 = np.array([0.0])        # referencia para u (vector 1x1)
ocp.model.cost_y_expr_0 = model.u

# --- Costo de trayectoria (stages 1..N-1) ---
ny = nx + nu   # 5
W = np.block([[W_x, np.zeros((nx, nu))],
              [np.zeros((nu, nx)), W_u]])
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.cost.W = W
ocp.cost.yref = np.zeros(ny)            # [0, pi, 0, 0, 0] implícito
ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)

# --- Costo terminal (stage N) - OPCIONAL (descomentar si se desea) ---
# ny_e = nx
# ocp.cost.cost_type_e = 'NONLINEAR_LS'
# ocp.cost.W_e = W_x
# ocp.cost.yref_e = np.array([0.0, np.pi, 0.0, 0.0])
# ocp.model.cost_y_expr_e = model.x

# ============ RESTRICCIONES RECTANGULARES (BGH) ============
ocp.constraints.constr_type = 'BGH'

# --- Límites de CONTROL ---
U_max = 80.0
ocp.constraints.idxbu = np.array([0], dtype=np.int32)   # índice 0 (única entrada)
ocp.constraints.lbu = np.array([-U_max])
ocp.constraints.ubu = np.array([ U_max])

# --- Límites de ESTADO (trayectoria) ---
x_max = 2.5
theta_max = np.deg2rad(60)          # ±60° alrededor de pi
x_dot_max = 3.0
theta_dot_max = np.deg2rad(180)

ocp.constraints.idxbx = np.array([0, 1, 2, 3], dtype=np.int32)
ocp.constraints.lbx = np.array([-x_max, np.pi - theta_max, -x_dot_max, -theta_dot_max])
ocp.constraints.ubx = np.array([ x_max, np.pi + theta_max,  x_dot_max,  theta_dot_max])

# --- Estado inicial FIJO (stage 0) ---
ocp.constraints.idxbx_0 = np.array([0, 1, 2, 3], dtype=np.int32)
ocp.constraints.lbx_0 = x0
ocp.constraints.ubx_0 = x0

# --- Límites terminales (stage N) ---
ocp.constraints.idxbx_e = np.array([0, 1, 2, 3], dtype=np.int32)
ocp.constraints.lbx_e = ocp.constraints.lbx
ocp.constraints.ubx_e = ocp.constraints.ubx

# ============ OPCIONES DEL SOLVER ============
ocp.solver_options.N_horizon = N
ocp.solver_options.tf = T
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.qp_solver_cond_N = 5
ocp.solver_options.qp_solver_warm_start = 2
ocp.solver_options.nlp_solver_max_iter = 10
ocp.solver_options.qp_solver_iter_max = 20
ocp.solver_options.print_level = 1
ocp.solver_options.ext_fun_compile_flags = '-O2 -fPIC'
ocp.solver_options.compile_interface = 'STATIC'

# ============ CREACIÓN DEL SOLVER ============
try:
    ocp_solver = AcadosOcpSolver(ocp, json_file='cart_pole_ocp.json')
    print("✓ Solver creado exitosamente con restricciones rectangulares")
except Exception as e:
    print(f"✗ Error creando solver: {e}")
    raise