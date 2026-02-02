"""
ocp_cartpole.py
Configuración del problema de control óptimo (OCP) para Acados
"""

import numpy as np
import casadi as ca
from acados_template import AcadosOcp
from model_cartpole import model_cartpole

def create_ocp():
    """
    Crea el problema de control óptimo (OCP) para NMPC del cartpole
    """
    ocp = AcadosOcp()

    # =========================
    # Modelo
    # =========================
    model = model_cartpole()
    ocp.model = model

    # =========================
    # Expresiones simbólicas para NONLINEAR_LS (¡OBLIGATORIO!)
    # =========================
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)  # [x; u] para k=0..N-1
    ocp.model.cost_y_expr_e = model.x                     # [x] para k=N

    # Obtener dimensiones del modelo
    nx = model.x.shape[0]  # 4
    nu = model.u.shape[0]  # 1
    ny = nx + nu           # 5
    ny_e = nx              # 4

    # =========================
    # Horizonte temporal
    # =========================
    Tf = 2.0
    N = 40
    
    ocp.solver_options.tf = Tf
    ocp.solver_options.N_horizon = N

    # =========================
    # Dimensiones (sin ny_0 - se infiere automáticamente)
    # =========================
    ocp.dims.nx = nx
    ocp.dims.nu = nu
    ocp.dims.ny = ny      
    ocp.dims.ny_e = ny_e
    ocp.dims.np = model.p.shape[0]  # Parámetros

    # =========================
    # Función de costo - NONLINEAR_LS (correcto y limpio)
    # =========================
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    Q = np.diag([100.0, 1000.0, 1.0, 10.0])
    R = np.diag([0.1])

    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W_e = Q

    x_ref = np.array([0.0, 0.0, 0.0, 0.0])
    u_ref = np.array([0.0])

    ocp.cost.yref = np.hstack((x_ref, u_ref))
    ocp.cost.yref_e = x_ref

    # ⚠️ NOTA: Con NONLINEAR_LS NO se usan Vx, Vu, Vx_e, W_0, etc.
    #          ¡TODAS esas matrices se eliminan! El coste se define
    #          mediante cost_y_expr y cost_y_expr_e.

    # =========================
    # Restricciones
    # =========================
    ocp.constraints.x0 = np.zeros(nx)

    F_max = 100.0
    ocp.constraints.lbu = np.array([-F_max])
    ocp.constraints.ubu = np.array([+F_max])
    ocp.constraints.idxbu = np.array([0])

    x_max = 0.45
    ocp.constraints.lbx = np.array([-x_max])
    ocp.constraints.ubx = np.array([+x_max])
    ocp.constraints.idxbx = np.array([0])

    # =========================
    # Opciones del solver
    # =========================
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.num_stages = 4
    ocp.solver_options.num_steps = 1

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tol = 1e-4

    # =========================
    # Parámetros físicos
    # =========================
    ocp.parameter_values = np.array([
        7.7,   # m_c
        0.7,   # m_p
        0.3,   # l
        0.02,  # I_p
        9.81   # g
    ])

    return ocp


if __name__ == "__main__":
    print("="*60)
    print("Probando creación del OCP")
    print("="*60)
    
    ocp = create_ocp()
    
    print(f"✅ OCP creado correctamente")
    print(f"   Modelo: {ocp.model.name}")
    print(f"   Horizonte: {ocp.solver_options.tf} s")
    print(f"   N pasos: {ocp.solver_options.N_horizon}")
    print(f"   nx: {ocp.dims.nx}")
    print(f"   nu: {ocp.dims.nu}")
    print(f"   np: {ocp.dims.np}")
    print(f"   ny: {ocp.dims.ny}")
    print(f"   ny_e: {ocp.dims.ny_e}")
    
    ny = ocp.dims.ny
    ny_e = ocp.dims.ny_e
    nx = ocp.dims.nx
    nu = ocp.dims.nu
    
    print(f"\n📊 Dimensiones de matrices:")
    print(f"   W: {ocp.cost.W.shape} (esperado: {ny}x{ny})")
    print(f"   W_e: {ocp.cost.W_e.shape} (esperado: {ny_e}x{ny_e})")
    print(f"   cost_y_expr: {ocp.model.cost_y_expr.shape} (esperado: ({ny}, 1))")
    print(f"   cost_y_expr_e: {ocp.model.cost_y_expr_e.shape} (esperado: ({ny_e}, 1))")
    
    print(f"\n✅ OCP listo!")
    print("="*60)