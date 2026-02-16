import mujoco
import mujoco.viewer
import time
import numpy as np
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

model = mujoco.MjModel.from_xml_path(
    "cart_pole_description/cartpole.xml"
)
data = mujoco.MjData(model)

# timestep real
dt = model.opt.timestep

# ============================
# LEER DATOS DEL PÉNDULO (UNA VEZ)
# ============================
pole_body_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "pole_link"
)

pole_mass = model.body_mass[pole_body_id]
pole_com_local = model.body_ipos[pole_body_id]       # [x, y, z]
pole_inertia_diag = model.body_inertia[pole_body_id] # [Ixx, Iyy, Izz]

# eje de rotación: Y
I_p = pole_inertia_diag[1]
l = abs(pole_com_local[2])

print("\n=== DATOS DEL PÉNDULO (MuJoCo) ===")
print(f"Masa total m_p      = {pole_mass:.4f} kg")
print(f"Centro de masas COM = {pole_com_local} [m]")
print(f"l (|z_COM|)         = {l:.4f} m")
print(f"Inercia Iyy         = {I_p:.6f} kg·m²")
print("=================================\n")

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        # --- POLE ---
        jid = model.joint("revolute_pole").id
        qpos_addr = model.jnt_qposadr[jid]

        # data.qpos[qpos_addr] = 3.14

        # avanzar simulación
        mujoco.mj_step(model, data)

        # Después de mj_step, imprime el número de contactos
        print("Número de contactos:", data.ncon)
        for i in range(data.ncon):
            c = data.contact[i]
            geom1_name = model.geom(c.geom1).name
            geom2_name = model.geom(c.geom2).name
            body1_name = model.body(model.geom_bodyid[c.geom1]).name
            body2_name = model.body(model.geom_bodyid[c.geom2]).name
            print(f"  Contacto entre geometría '{geom1_name}' (cuerpo '{body1_name}') y "
                f"geometría '{geom2_name}' (cuerpo '{body2_name}')")

        # --- POLE ---
        jid = model.joint("revolute_pole").id
        qpos_addr = model.jnt_qposadr[jid]
        qvel_addr = model.jnt_dofadr[jid]

        angle = data.qpos[qpos_addr]
        angvel = data.qvel[qvel_addr]

        # --- CART ---
        jid = model.joint("cart_slide").id
        x = data.qpos[model.jnt_qposadr[jid]]
        xdot = data.qvel[model.jnt_dofadr[jid]]

        print(
            f"x={x:+.3f}, xdot={xdot:+.3f}, "
            f"theta={angle:+.3f}, thetadot={angvel:+.3f}"
        )

        # ⏱️ correr en tiempo real
        viewer.sync()
        time.sleep(dt)
