"""
model_cartpole.py
Modelo del péndulo invertido sobre carrito para Acados
"""

from acados_template import AcadosModel
import casadi as ca

def model_cartpole():
    """
    Modelo del péndulo invertido sobre carrito
    Estados: x = [x, theta, x_dot, theta_dot]
    Control: u = [F]
    Parámetros: p = [m_c, m_p, l, I_p, g]
    """
    
    # Crear objeto AcadosModel (NO un diccionario)
    model = AcadosModel()
    model.name = "cartpole_mujoco"
    
    # =========================
    # Parámetros simbólicos
    # =========================
    m_c = ca.SX.sym("m_c")   # masa carro [kg]
    m_p = ca.SX.sym("m_p")   # masa péndulo [kg]
    l   = ca.SX.sym("l")     # longitud al COM péndulo [m]
    I_p = ca.SX.sym("I_p")   # inercia péndulo (eje hinge) [kg⋅m²]
    g   = ca.SX.sym("g")     # gravedad [m/s²]

    p = ca.vertcat(m_c, m_p, l, I_p, g)

    # =========================
    # Estados
    # =========================
    x      = ca.SX.sym("x")          # posición del carro [m]
    theta  = ca.SX.sym("theta")      # ángulo del péndulo [rad]
    x_dot  = ca.SX.sym("x_dot")      # velocidad del carro [m/s]
    th_dot = ca.SX.sym("theta_dot")  # velocidad angular [rad/s]

    states = ca.vertcat(x, theta, x_dot, th_dot)

    # =========================
    # Control
    # =========================
    F = ca.SX.sym("F")  # fuerza aplicada [N]
    controls = ca.vertcat(F)

    # =========================
    # Dinámica
    # =========================
    sin_t = ca.sin(theta)
    cos_t = ca.cos(theta)

    # Denominador común
    denom = m_c + m_p - (m_p**2 * l**2 * cos_t**2) / (I_p + m_p * l**2)

    # Aceleración del carro
    x_ddot = (
        F
        + m_p * l * sin_t * th_dot**2
        - m_p * g * sin_t * cos_t
    ) / denom

    # Aceleración angular del péndulo
    theta_ddot = (
        m_p * g * l * sin_t
        - m_p * l * cos_t * x_ddot
    ) / (I_p + m_p * l**2)

    # Ecuaciones de estado explícitas: dx/dt = f(x, u, p)
    f_expl = ca.vertcat(
        x_dot,
        th_dot,
        x_ddot,
        theta_ddot
    )

    # =========================
    # Forma implícita (requerida por algunos integradores)
    # =========================
    # Variables para las derivadas temporales
    x_dot_sym      = ca.SX.sym("x_dot_sym")
    theta_dot_sym  = ca.SX.sym("theta_dot_sym")
    x_ddot_sym     = ca.SX.sym("x_ddot_sym")
    theta_ddot_sym = ca.SX.sym("theta_ddot_sym")
    
    xdot = ca.vertcat(x_dot_sym, theta_dot_sym, x_ddot_sym, theta_ddot_sym)
    
    # Forma implícita: 0 = xdot - f(x, u, p)
    f_impl = xdot - f_expl

    # =========================
    # Asignar al modelo de Acados
    # =========================
    model.x = states
    model.xdot = xdot
    model.u = controls
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    return model  # Retornar el objeto AcadosModel, NO un diccionario


# Test del modelo
if __name__ == "__main__":
    print("="*60)
    print("Probando creación del modelo del cartpole")
    print("="*60)
    
    model = model_cartpole()
    
    print(f"✅ Modelo creado: {model.name}")
    print(f"   Estados (nx):     {model.x.size()[0]}")
    print(f"   Controles (nu):   {model.u.size()[0]}")
    print(f"   Parámetros (np):  {model.p.size()[0]}")
    print(f"\n✅ El modelo está listo para Acados!")
    print("="*60)