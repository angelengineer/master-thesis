import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# ==========================================================
# CONFIGURACIÓN
# ==========================================================

XML_PATH = "cart_pole_description/cartpole.xml"

NEW_SPHERE_RADIUS = 0.02    # cambiar tamaño esfera
NEW_SPHERE_MASS = 0.5        # masa de la esfera (si se usa)
EXTRA_POLE_MASS = 0         # masa extra al péndulo
CART_DAMPING = 0.8            # damping cart
POLE_DAMPING = 0            # damping pole
CART_FRICTION = 0.0           # fricción tipo Coulomb
ENABLE_WIND = False            # activar viento

# ==========================================================
# CARGAR MODELO
# ==========================================================

os.chdir(os.path.dirname(os.path.abspath(__file__)))

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

dt = model.opt.timestep

# ==========================================================
# FUNCIONES AUXILIARES
# ==========================================================

def print_pole_info(model):
    pole_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "pole_link"
    )

    mass = model.body_mass[pole_body_id]
    com = model.body_ipos[pole_body_id]
    inertia = model.body_inertia[pole_body_id]

    print("\n=== DATOS DEL PÉNDULO ===")
    print(f"Masa total       : {mass:.4f} kg")
    print(f"Centro de masas  : {com}")
    print(f"Inercia diagonal : {inertia}")
    print("==========================\n")


def apply_model_uncertainty(model):
    # -------- Extra Sphere radius --------
    sphere_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "sphere"
    )
    model.geom_size[sphere_id][0] = NEW_SPHERE_RADIUS


    # -------- Extra sphere  mass --------
    extra_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "extra_mass"
    )
    model.body_mass[extra_body_id] += NEW_SPHERE_MASS

    # -------- Extra pendulum  mass --------
    pendulum_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "pole_link"
    )
    model.body_mass[pendulum_id] += EXTRA_POLE_MASS

    # -------- Joint damping & friction --------
    cart_joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "cart_slide"
    )
    pole_joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "revolute_pole"
    )

    model.dof_damping[cart_joint_id] = CART_DAMPING
    model.dof_damping[pole_joint_id] = POLE_DAMPING
    model.dof_frictionloss[cart_joint_id] = CART_FRICTION

    mujoco.mj_forward(model, data)


# ==========================================================
# ANTES DE MODIFICAR
# ==========================================================

print("ANTES DE MODIFICAR:")
print_pole_info(model)

# ==========================================================
# APLICAR CAMBIOS
# ==========================================================

apply_model_uncertainty(model)

print("DESPUÉS DE MODIFICAR:")
print_pole_info(model)

# ==========================================================
# SIMULACIÓN
# ==========================================================

pole_body_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_BODY, "pole_link"
)

with mujoco.viewer.launch_passive(model, data) as viewer:

    start_time = time.time()

    while viewer.is_running():

        # -------- Viento opcional --------
        if ENABLE_WIND:
            t = time.time() - start_time
            wind_force = 5.0 * np.sin(1.0 * t)
            data.xfrc_applied[pole_body_id, 0] = wind_force
        else:
            data.xfrc_applied[pole_body_id, :] = 0.0

        mujoco.mj_step(model, data)

        viewer.sync()
        time.sleep(dt)
