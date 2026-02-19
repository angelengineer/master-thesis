import mujoco
import mujoco.viewer
import time
import numpy as np
import os
from acados_template import AcadosOcpSolver
from cart_pole_ocp import W_u, ocp   # Importa el objeto ocp definido en tu generador

# Cambiar al directorio del script
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ============================================================
# 1. CARGAR MODELO MUJOCO
# ============================================================
model = mujoco.MjModel.from_xml_path("cart_pole_description/cartpole.xml")
data = mujoco.MjData(model)
dt = model.opt.timestep
# ============================================================
# APLICAR MISMATCH (MODELO REAL ≠ MODELO NMPC)
# ============================================================

def apply_model_uncertainty(model, data):
    # ---- Sphere size ----
    sphere_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "sphere")
    model.geom_size[sphere_id][0] = 0.03  # radio diferente

    # ---- Extra sphere mass ----
    extra_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "extra_mass")
    model.body_mass[extra_body_id] = 4.5  # masa diferente

    # ---- Joint damping ----
    cart_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cart_slide")
    pole_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolute_pole")

    model.dof_damping[cart_joint_id] = 0.2
    model.dof_damping[pole_joint_id] = 0.005
    model.dof_frictionloss[cart_joint_id] = 0.2

    mujoco.mj_forward(model, data)

apply_model_uncertainty(model, data)


# ============================================================
# 2. CARGAR SOLVER ACADOS (SIN REGENERAR)
# ============================================================
ocp_solver = AcadosOcpSolver(ocp, json_file='cart_pole_ocp_augmented.json', generate=False)
N_horizon = 40

# ============================================================
# 3. CONFIGURACIÓN DE CONTROL
# ============================================================
# Convención: péndulo arriba = π rad, abajo = 0 rad
x0 = np.array([0.0, 0.0, 0.0, 0.0])   # estado inicial: péndulo abajo
x_ref = np.array([0.0, np.pi, 0.0, 0.0])  # referencia: péndulo arriba

# Establecer estado inicial en MuJoCo
jid_cart = model.joint("cart_slide").id
jid_pole = model.joint("revolute_pole").id
data.qpos[model.jnt_qposadr[jid_cart]] = x0[0]
data.qpos[model.jnt_qposadr[jid_pole]] = x0[1]
data.qvel[model.jnt_dofadr[jid_cart]] = x0[2]
data.qvel[model.jnt_dofadr[jid_pole]] = x0[3]

# Variables de control
control_active = True   # Activado desde el principio
sim_time = 0.0
control_period = 0.025  # 40 Hz

# Límites amplios para relajar las restricciones de estado en stages futuros
lbx = np.array([-2.5, -10*np.pi, -3.0, -10*np.pi])   # [x, theta, x_dot, theta_dot]
ubx = np.array([ 2.5,  10*np.pi,  3.0,  10*np.pi])

# Límites de control
lbu = -150.0
ubu =  150.0

# Pesos para la función de costo (ajustar según preferencia)
W_x = np.diag([1e3, 1e3, 1e-1, 1e-2])   
W = np.block([[W_x, np.zeros((4,1))],[np.zeros((1,4)), W_u]])

# Para mantener el último control exitoso (en caso de fallo del solver)
last_successful_u = 0.0

print("\n" + "="*60)
print("CONTROL MPC PÉNDULO INVERTIDO")
print("="*60)
print("Control activado automáticamente")
print("Presiona ESC en la ventana del visor para salir")
print("="*60 + "\n")

# ============================================================
# 4. LOOP DE SIMULACIÓN CON MPC
# ============================================================
with mujoco.viewer.launch_passive(model, data) as viewer:
    last_control_time = -control_period
    
    while viewer.is_running():
        # Leer estado actual de MuJoCo
        x = data.qpos[model.jnt_qposadr[jid_cart]]
        theta = data.qpos[model.jnt_qposadr[jid_pole]]
        x_dot = data.qvel[model.jnt_dofadr[jid_cart]]
        theta_dot = data.qvel[model.jnt_dofadr[jid_pole]]
        x_current = np.array([x, theta, x_dot, theta_dot])
        
        # ====================================================
        # CALCULAR CONTROL MPC (a frecuencia definida)
        # ====================================================
        if control_active and (sim_time - last_control_time >= control_period):
            
            # PASO 1: Fijar estado inicial (stage 0)
            ocp_solver.set(0, "lbx", x_current)
            ocp_solver.set(0, "ubx", x_current)
            
            # PASO 2: Setear constraints y pesos
            for i in range(1,N_horizon):
                ocp_solver.constraints_set(i, "lbx", lbx)
                ocp_solver.constraints_set(i, "ubx", ubx)

            
            for i in range(N_horizon):   # incluye etapa terminal
                if i == 0:
                    ocp_solver.cost_set(i, "W", W_u)

                    continue   # saltar etapa 0
                if i == N_horizon:
                    ocp_solver.cost_set(i, "W", W_x)   # terminal
                else:
                    ocp_solver.cost_set(i, "W", W)

            for i in range(N_horizon):
                ocp_solver.constraints_set(i, "lbu", np.array([lbu]))
                ocp_solver.constraints_set(i, "ubu", np.array([ubu]))

            # PASO 4: Configurar referencias
            # Stage 0: solo control deseado (dimensión 1)
            ocp_solver.set(0, "yref", np.array([0.0]))
            # Stages 1..N-1: estado + control deseados (dimensión 5)
            for i in range(1, N_horizon):
                y_ref = np.concatenate([x_ref, [0.0]])  # [x, theta, x_dot, theta_dot, u]
                ocp_solver.set(i, "yref", y_ref)
            
            # PASO 5: Resolver OCP
            status = ocp_solver.solve()
            
            # PASO 6: Aplicar control
            if status == 0:
                u_opt = ocp_solver.get(0, "u")
                data.ctrl[0] = u_opt[0]
                last_successful_u = u_opt[0]   # guardar para posible fallo futuro
                print(f"t={sim_time:6.2f}s | x={x:+6.3f} | θ={np.rad2deg(theta):+7.2f}° | u={u_opt[0]:+7.2f} N")
            else:
                print(f"⚠️  Solver warning: status = {status} (aplicando último control exitoso: {last_successful_u:.2f})")
                data.ctrl[0] = last_successful_u
            
            last_control_time = sim_time
        
        # Avanzar simulación
        mujoco.mj_step(model, data)
        sim_time += dt
        
        # Sincronizar visualización
        viewer.sync()
        # Pequeña pausa para tiempo real (opcional)
        time.sleep(dt)

print("\nSimulación finalizada.")