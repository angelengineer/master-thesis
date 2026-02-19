# cart_pole_ocp_augmented.py
import casadi as ca
import numpy as np
from acados_template import AcadosModel
from residual_nn import build_residual_nn  # Tu implementación con normalización quemada

def cart_pole_model_augmented(**kwargs):
    """
    Modelo híbrido: dinámica nominal + corrección residual aprendida.
    Convención de ángulo: θ = 0 rad → péndulo colgando (abajo), θ = π rad → invertido (arriba)
    ¡Alineado 100% con tu modelo MuJoCo!
    """
    # ------- Parámetros físicos NOMINALES (coinciden con tu XML MuJoCo) -------
    M  = 7.67      # kg - masa carrito
    m  = 0.7       # kg - masa péndulo
    L  = 0.06      # m  - distancia pivote → centro de masa
    I  = 3.9750e-3 # kg·m² - inercia respecto al centro de masa
    dz = 0.0       # amortiguamiento nominal (0 = sin fricción)
    dy = 0.0       # fricción Coulomb nominal (0 = sin fricción)
    g  = 9.81      # m/s²
    delta_t = None

    # ------- Procesar argumentos opcionales (para compatibilidad) -------
    for key, val in kwargs.items():
        if key.lower() == 'm':       m = val
        elif key.lower() == 'M':     M = val
        elif key.lower() == 'l':     L = val
        elif key.lower() == 'i':     I = val
        elif key.lower() == 'dz':    dz = val
        elif key.lower() == 'dy':    dy = val
        elif key.lower() == 'g':     g = val
        elif key.lower() == 'delta_t': delta_t = val

    # ------- Momento de inercia respecto al pivote -------
    J = m * L**2 + I  # kg·m²

    # ------- Variables simbólicas -------
    p      = ca.SX.sym('p')       # posición carrito [m]
    theta  = ca.SX.sym('theta')   # ángulo péndulo [rad] → 0 = colgando, π = invertido
    v      = ca.SX.sym('v')       # velocidad carrito [m/s]
    omega  = ca.SX.sym('omega')   # velocidad angular [rad/s]
    x = ca.vertcat(p, theta, v, omega)
    nx = 4

    xdot = ca.SX.sym('xdot', nx, 1)  # derivadas para forma implícita
    F = ca.SX.sym('F')               # fuerza de control [N]
    u = F
    nu = 1

    # ------- Trigonometría -------
    s = ca.sin(theta)
    c = ca.cos(theta)

    # ============================================================
    # DINÁMICA NOMINAL (Euler-Lagrange) - ¡MISMA QUE TU MODELO ACADOS ORIGINAL!
    # ============================================================
    M11 = M + m
    M12 = m * L * c
    M22 = J
    det = M11 * M22 - M12 * M12

    rhs1 = F + m * L * s * omega**2 - dz * v - dy * ca.sign(v)  # términos de fricción NOMINALES (0 en tu caso)
    rhs2 = -m * g * L * s - dz * omega  # amortiguamiento nominal en eje (0 en tu caso)

    a_nom     = (rhs1 * M22 - rhs2 * M12) / det   # aceleración carrito nominal
    alpha_nom = (rhs2 * M11 - rhs1 * M12) / det   # aceleración angular nominal

    # ============================================================
    # CORRECCIÓN RESIDUAL APRENDIDA (¡NORMALIZACIÓN YA INCLUIDA!)
    # ============================================================
    residual_nn = build_residual_nn()  # Retorna [Δa, Δα] en ESCALA FÍSICA
    delta_acc = residual_nn(x, u)      # Evaluación directa con estado físico
    
    # Extraer componentes del residual
    delta_a = delta_acc[0]     # Corrección para aceleración del carrito
    delta_alpha = delta_acc[1] # Corrección para aceleración angular

    # ============================================================
    # DINÁMICA HÍBRIDA (nominal + residual)
    # ============================================================
    a_hybrid     = a_nom + delta_a
    alpha_hybrid = alpha_nom + delta_alpha

    f_hybrid = ca.vertcat(
        v,              # dp/dt = v
        omega,          # dtheta/dt = omega
        a_hybrid,       # dv/dt = a_nom + Δa
        alpha_hybrid    # domega/dt = alpha_nom + Δα
    )

    f_impl_expr = f_hybrid - xdot

    # ------- Dinámica discreta (opcional) -------
    if delta_t is not None:
        disc_dyn_expr = x + delta_t * f_hybrid
    else:
        disc_dyn_expr = None

    # ------- Crear modelo Acados -------
    model = AcadosModel()
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = None
    model.p = None
    model.f_expl_expr = f_hybrid
    model.f_impl_expr = f_impl_expr
    model.disc_dyn_expr = disc_dyn_expr
    model.name = 'cart_pole_augmented'

    # Etiquetas para debugging
    model.x_labels = ['$p$ [m]', r'$\theta$ [rad]', '$v$ [m/s]', r'$\omega$ [rad/s]']
    model.u_labels = ['$F$ [N]']
    model.t_label = '$t$ [s]'

    return model

