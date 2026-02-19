import casadi as ca
import numpy as np
from acados_template import AcadosModel

def cart_pole_model(**kwargs):
    """
    Crea el modelo Acados para péndulo sobre carro con parámetros hardcodeados.
    Versión explícita (sin ca.solve) → compatible con GNSF.
    Parámetros opcionales (keyword arguments):
        'M', 'm', 'L', 'I', 'dz', 'dy', 'g', 'delta_t'
    Retorna:
        model: AcadosModel listo para usar en OCP/MHE.
    """
    # ------- Valores por defecto -------
    M  = 7.67
    m  = 0.7
    L  = 0.06
    I  = 3.9750e-03
    dz = 0.0
    dy = 0.0
    g  = 9.81
    delta_t = None

    # ------- Procesar argumentos opcionales -------
    for key, val in kwargs.items():
        if key.lower() == 'm':       m = val
        elif key.lower() == 'M':     M = val
        elif key.lower() == 'l':     L = val
        elif key.lower() == 'i':     I = val
        elif key.lower() == 'dz':    dz = val
        elif key.lower() == 'dy':    dy = val
        elif key.lower() == 'g':     g = val
        elif key.lower() == 'delta_t': delta_t = val
        else:
            print(f'Advertencia: parámetro "{key}" desconocido, se ignora.')

    # ------- Momento de inercia respecto al pivote -------
    J = m * L**2 + I   # valor numérico

    # ------- Variables simbólicas del estado -------
    p      = ca.SX.sym('p')
    theta  = ca.SX.sym('theta')
    v      = ca.SX.sym('v')
    omega  = ca.SX.sym('omega')
    x = ca.vertcat(p, theta, v, omega)
    nx = 4

    # Derivadas del estado (para forma implícita)
    xdot = ca.SX.sym('xdot', nx, 1)

    # ------- Entrada de control -------
    F = ca.SX.sym('F')
    u = F
    nu = 1

    # ------- Trigonometría -------
    s = ca.sin(theta)
    c = ca.cos(theta)

    # ------- Matriz de masa y vector rhs (solo para referencia, no se usa ca.solve) -------
    M11 = M + m
    M12 = m * L * c
    M22 = J
    det = M11 * M22 - M12 * M12   # determinante

    rhs1 = F + m * L * s * omega**2
    rhs2 = - m * g * L * s

    # ------- Aceleraciones en forma explícita (cerrada) -------
    a     = (rhs1 * M22 - rhs2 * M12) / det
    alpha = (rhs2 * M11 - rhs1 * M12) / det

    # ------- Dinámica continua -------
    f_expl_expr = ca.vertcat(v, omega, a, alpha)
    f_impl_expr = f_expl_expr - xdot

    # ------- Dinámica discreta (opcional) -------
    if delta_t is not None:
        disc_dyn_expr = x + delta_t * f_expl_expr
    else:
        disc_dyn_expr = None   # IMPORTANTE: usar None, no ca.DM([])

    # ------- Crear estructura AcadosModel -------
    model = AcadosModel()
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = None
    model.p = None
    model.f_expl_expr = f_expl_expr
    model.f_impl_expr = f_impl_expr
    model.disc_dyn_expr = disc_dyn_expr
    model.name = 'cart_pole'

    # Etiquetas (opcional pero útil)
    model.x_labels = ['$x$ [m]', r'$\theta$ [rad]', '$v$ [m/s]', r'$\omega$ [rad/s]']
    model.u_labels = ['$F$ [N]']
    model.t_label = '$t$ [s]'

    return model