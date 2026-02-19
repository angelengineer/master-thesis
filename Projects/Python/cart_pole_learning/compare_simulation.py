import mujoco
import mujoco.viewer
import time
import numpy as np
import os
from acados_template import AcadosOcpSolver
from cart_pole_ocp import W_u as W_u_nom, ocp as ocp_nom
from cart_pole_ocp_augmented import W_u as W_u_aug, ocp as ocp_aug

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ============================================================
# 1. CARGAR MODELO MUJOCO (DUAL)
# ============================================================
model = mujoco.MjModel.from_xml_path("cart_pole_description/cartpole_dual.xml")
data = mujoco.MjData(model)
dt = model.opt.timestep

# ============================================================
# APLICAR MISMATCH A AMBOS SISTEMAS
# ============================================================

def apply_model_uncertainty(model, suffix):

    sphere_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"sphere_{suffix}")
    model.geom_size[sphere_id][0] = 0.03

    extra_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"extra_mass_{suffix}")
    model.body_mass[extra_body_id] = 2.5

    cart_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"cart_slide_{suffix}")
    pole_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"revolute_pole_{suffix}")

    model.dof_damping[cart_joint_id] = 0.2
    model.dof_damping[pole_joint_id] = 0.005
    model.dof_frictionloss[cart_joint_id] = 0.2

apply_model_uncertainty(model, "1")
apply_model_uncertainty(model, "2")
mujoco.mj_forward(model, data)

# ============================================================
# 2. IDS
# ============================================================

jid_cart1 = model.joint("cart_slide_1").id
jid_pole1 = model.joint("revolute_pole_1").id
jid_cart2 = model.joint("cart_slide_2").id
jid_pole2 = model.joint("revolute_pole_2").id

act1 = model.actuator("cart_force_1").id
act2 = model.actuator("cart_force_2").id

# ============================================================
# 3. SOLVERS
# ============================================================

ocp_solver_nom = AcadosOcpSolver(ocp_nom, json_file='cart_pole_ocp.json', generate=False)
ocp_solver_aug = AcadosOcpSolver(ocp_aug, json_file='cart_pole_ocp_augmented.json', generate=False)

N_horizon = 40

# ============================================================
# 4. CONFIGURACIÓN
# ============================================================

x0 = np.array([0.0, 0.0, 0.0, 0.0])
x_ref = np.array([0.0, np.pi, 0.0, 0.0])

# Inicializar ambos sistemas
for jid_cart, jid_pole in [(jid_cart1, jid_pole1), (jid_cart2, jid_pole2)]:
    data.qpos[model.jnt_qposadr[jid_cart]] = x0[0]
    data.qpos[model.jnt_qposadr[jid_pole]] = x0[1]
    data.qvel[model.jnt_dofadr[jid_cart]] = x0[2]
    data.qvel[model.jnt_dofadr[jid_pole]] = x0[3]

mujoco.mj_forward(model, data)

control_period = 0.025
sim_time = 0.0
last_control_time = -control_period

lbx = np.array([-2.5, -10*np.pi, -3.0, -10*np.pi])
ubx = np.array([ 2.5,  10*np.pi,  3.0,  10*np.pi])
lbu = -150.0
ubu =  150.0

W_x = np.diag([1e3, 1e3, 1e-1, 1e-2])
W_nom = np.block([[W_x, np.zeros((4,1))],[np.zeros((1,4)), W_u_nom]])
W_aug = np.block([[W_x, np.zeros((4,1))],[np.zeros((1,4)), W_u_aug]])

last_u_nom = 0.0
last_u_aug = 0.0

print("\n============================================================")
print("COMPARACIÓN NMPC NOMINAL vs AUGMENTED")
print("============================================================\n")

# ============================================================
# 5. LOOP PRINCIPAL
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # Estados sistema 1
        x1 = data.qpos[model.jnt_qposadr[jid_cart1]]
        th1 = data.qpos[model.jnt_qposadr[jid_pole1]]
        xd1 = data.qvel[model.jnt_dofadr[jid_cart1]]
        thd1 = data.qvel[model.jnt_dofadr[jid_pole1]]
        state1 = np.array([x1, th1, xd1, thd1])

        # Estados sistema 2
        x2 = data.qpos[model.jnt_qposadr[jid_cart2]]
        th2 = data.qpos[model.jnt_qposadr[jid_pole2]]
        xd2 = data.qvel[model.jnt_dofadr[jid_cart2]]
        thd2 = data.qvel[model.jnt_dofadr[jid_pole2]]
        state2 = np.array([x2, th2, xd2, thd2])

        if sim_time - last_control_time >= control_period:

            # ================= NOMINAL =================
            ocp_solver_nom.set(0, "lbx", state1)
            ocp_solver_nom.set(0, "ubx", state1)

            for i in range(1, N_horizon):
                ocp_solver_nom.constraints_set(i, "lbx", lbx)
                ocp_solver_nom.constraints_set(i, "ubx", ubx)

            for i in range(N_horizon):
                ocp_solver_nom.constraints_set(i, "lbu", np.array([lbu]))
                ocp_solver_nom.constraints_set(i, "ubu", np.array([ubu]))

            for i in range(N_horizon):
                if i == 0:
                    ocp_solver_nom.cost_set(i, "W", W_u_nom)
                else:
                    ocp_solver_nom.cost_set(i, "W", W_nom)

            ocp_solver_nom.set(0, "yref", np.array([0.0]))
            for i in range(1, N_horizon):
                ocp_solver_nom.set(i, "yref", np.concatenate([x_ref, [0.0]]))

            status1 = ocp_solver_nom.solve()
            if status1 == 0:
                u1 = ocp_solver_nom.get(0, "u")[0]
                last_u_nom = u1
            else:
                u1 = last_u_nom

            # ================= AUGMENTED =================
            ocp_solver_aug.set(0, "lbx", state2)
            ocp_solver_aug.set(0, "ubx", state2)

            for i in range(1, N_horizon):
                ocp_solver_aug.constraints_set(i, "lbx", lbx)
                ocp_solver_aug.constraints_set(i, "ubx", ubx)

            for i in range(N_horizon):
                ocp_solver_aug.constraints_set(i, "lbu", np.array([lbu]))
                ocp_solver_aug.constraints_set(i, "ubu", np.array([ubu]))

            for i in range(N_horizon):
                if i == 0:
                    ocp_solver_aug.cost_set(i, "W", W_u_aug)
                else:
                    ocp_solver_aug.cost_set(i, "W", W_aug)

            ocp_solver_aug.set(0, "yref", np.array([0.0]))
            for i in range(1, N_horizon):
                ocp_solver_aug.set(i, "yref", np.concatenate([x_ref, [0.0]]))

            status2 = ocp_solver_aug.solve()
            if status2 == 0:
                u2 = ocp_solver_aug.get(0, "u")[0]
                last_u_aug = u2
            else:
                u2 = last_u_aug

            data.ctrl[act1] = u1
            data.ctrl[act2] = u2

            print(f"t={sim_time:5.2f} | Nom θ={np.rad2deg(th1):+6.1f}° u={u1:+6.1f} | Aug θ={np.rad2deg(th2):+6.1f}° u={u2:+6.1f}")

            last_control_time = sim_time

        mujoco.mj_step(model, data)
        sim_time += dt
        viewer.sync()
        time.sleep(dt)

print("\nSimulación finalizada.")
