import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path(
    "../../Fusion/cartPole_description/mujoco/cartpole.xml"
)
data = mujoco.MjData(model)

# timestep real
dt = model.opt.timestep

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        # aplicar control
        #data.ctrl[0] = 1.0

        # avanzar simulación
        mujoco.mj_step(model, data)

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
