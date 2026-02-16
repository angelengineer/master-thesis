import mujoco
import numpy as np
import os
import time
import sys

# ==========================================================
# CONFIGURACIÓN DE DATASET
# ==========================================================
XML_PATH = "cart_pole_description/cartpole.xml"
SAVE_PATH = "cartpole_residual_dataset.npy"

NUM_EPISODES = 500      # Más episodios = mejor cobertura
MAX_STEPS = 1000             # Pasos máximos por episodio
CONTROL_RANGE = (-150, 150) # Rango de exploración (ajusta según límites físicos)
VISUALIZE = False           # ¡ACTIVA SOLO PARA DEBUG! (False para recolección rápida)
SEED = 42
np.random.seed(SEED)

# Rangos de incertidumbre REALISTAS (ajusta según tu modelo físico)
UNCERTAINTY_CONFIG = {
    "extra_mass_range": (0.0, 0.25),    # Masa extra en "extra_mass" body [kg]
    "cart_damping_range": (0.1, 1.5),   # Amortiguamiento carrito [Ns/m]
    "pole_damping_range": (0.01, 0.15), # Amortiguamiento péndulo [Ns/m]
    "cart_friction_range": (0.0, 0.8),  # Fricción Coulomb carrito [N]
    "wind_force_range": (0.0, 8.0),     # Magnitud máxima del viento [N]
    "wind_freq_range": (0.5, 3.0)       # Frecuencia del viento [rad/s]
}

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ==========================================================
# FUNCIONES CRÍTICAS (MEJORADAS Y ROBUSTAS)
# ==========================================================
def apply_random_uncertainties(model, data, config):
    """Aplica incertidumbres físicas ALEATORIAS al modelo MuJoCo (¡NO al XML!)"""
    try:
        # 1. Masa extra en el cuerpo "extra_mass" (¡NO sumar, SETEAR valor absoluto!)
        extra_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "extra_mass")
        random_extra_mass = np.random.uniform(*config["extra_mass_range"])
        model.body_mass[extra_body_id] = random_extra_mass
        
        # 2. Amortiguamiento y fricción en joints
        cart_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cart_slide")
        pole_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolute_pole")
        
        model.dof_damping[cart_joint_id] = np.random.uniform(*config["cart_damping_range"])
        model.dof_damping[pole_joint_id] = np.random.uniform(*config["pole_damping_range"])
        model.dof_frictionloss[cart_joint_id] = np.random.uniform(*config["cart_friction_range"])
        
        # 3. Parámetros de viento (se usarán durante simulación)
        wind_params = {
            'enabled': np.random.rand() > 0.3,  # 70% de episodios con viento
            'amp': np.random.uniform(0, config["wind_force_range"][1]),
            'freq': np.random.uniform(*config["wind_freq_range"]),
            'phase': np.random.uniform(0, 2*np.pi)
        }
        
        # ¡ACTUALIZAR CONSTANTES DERIVADAS! (CRÍTICO)
        mujoco.mj_forward(model, data)
        
        return {
            'extra_mass': random_extra_mass,
            'cart_damping': model.dof_damping[cart_joint_id],
            'pole_damping': model.dof_damping[pole_joint_id],
            'cart_friction': model.dof_frictionloss[cart_joint_id],
            'wind': wind_params
        }
    
    except Exception as e:
        print(f"❌ Error aplicando incertidumbres: {e}")
        print("Verifica que estos nombres existan en tu XML:")
        print("  - Cuerpo: 'extra_mass'")
        print("  - Joints: 'cart_slide', 'revolute_pole'")
        sys.exit(1)

def reset_to_random_state(model, data, cart_joint_id, pole_joint_id):
    """Inicializa estado aleatorio con cobertura inteligente del espacio de estados"""
    # 70% cerca de posición colgante (0 rad), 30% cerca de invertida (π rad)
    if np.random.rand() < 0.7:
        theta0 = np.random.uniform(-0.7, 0.7)  # Cerca de abajo
    else:
        theta0 = np.pi + np.random.uniform(-0.6, 0.6)  # Cerca de arriba (inestable)
    
    # Posición carrito y velocidades
    x0 = np.random.uniform(-1.2, 1.2)
    xdot0 = np.random.uniform(-1.5, 1.5)
    thetadot0 = np.random.uniform(-2.0, 2.0)
    
    # Aplicar al modelo
    data.qpos[model.jnt_qposadr[cart_joint_id]] = x0
    data.qpos[model.jnt_qposadr[pole_joint_id]] = theta0
    data.qvel[model.jnt_dofadr[cart_joint_id]] = xdot0
    data.qvel[model.jnt_dofadr[pole_joint_id]] = thetadot0
    mujoco.mj_forward(model, data)
    
    return np.array([x0, theta0, xdot0, thetadot0])

def get_state(model, data, cart_joint_id, pole_joint_id):
    """Extrae estado [x, θ, ẋ, θ̇] de forma robusta"""
    return np.array([
        data.qpos[model.jnt_qposadr[cart_joint_id]],
        data.qpos[model.jnt_qposadr[pole_joint_id]],
        data.qvel[model.jnt_dofadr[cart_joint_id]],
        data.qvel[model.jnt_dofadr[pole_joint_id]]
    ])

# ==========================================================
# VALIDACIÓN PREVIA (EVITA ERRORES COSTOSOS)
# ==========================================================
print("="*60)
print("🔍 VALIDANDO ESTRUCTURA DEL MODELO MUJOCO")
print("="*60)
test_model = mujoco.MjModel.from_xml_path(XML_PATH)
test_data = mujoco.MjData(test_model)

required_bodies = ["extra_mass", "pole_link", "cart"]
required_joints = ["cart_slide", "revolute_pole"]

for body in required_bodies:
    try:
        _ = mujoco.mj_name2id(test_model, mujoco.mjtObj.mjOBJ_BODY, body)
        print(f"✅ Cuerpo '{body}' encontrado")
    except:
        print(f"❌ Cuerpo '{body}' NO encontrado en XML")
        print("Solución: Ajusta los nombres en apply_random_uncertainties() o modifica tu XML")
        sys.exit(1)

for joint in required_joints:
    try:
        _ = mujoco.mj_name2id(test_model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        print(f"✅ Joint '{joint}' encontrado")
    except:
        print(f"❌ Joint '{joint}' NO encontrado en XML")
        sys.exit(1)

print("="*60)
print("✅ VALIDACIÓN EXITOSA - Iniciando recolección de datos")
print(f"Episodios: {NUM_EPISODES} | Pasos máx/episodio: {MAX_STEPS}")
print(f"Incertidumbres: Masa extra [{UNCERTAINTY_CONFIG['extra_mass_range'][0]}, {UNCERTAINTY_CONFIG['extra_mass_range'][1]}] kg")
print("="*60)

# ==========================================================
# RECOLECCIÓN DE DATOS
# ==========================================================
dataset = []
total_steps = 0
start_time = time.time()

for ep in range(NUM_EPISODES):
    # --- 1. CARGAR MODELO FRESCO (¡CLAVE PARA EVITAR CONTAMINACIÓN!) ---
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    
    # Obtener IDs de joints (necesarios tras recargar)
    cart_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cart_slide")
    pole_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "revolute_pole")
    pole_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pole_link")
    
    # --- 2. APLICAR INCERTIDUMBRES ALEATORIAS ---
    ep_params = apply_random_uncertainties(model, data, UNCERTAINTY_CONFIG)
    
    # --- 3. RESETEAR A ESTADO ALEATORIO ---
    reset_to_random_state(model, data, cart_joint_id, pole_joint_id)
    
    # --- 4. INICIAR VISUALIZACIÓN (SOLO SI DEBUG) ---
    viewer = None
    if VISUALIZE and ep == 0:
        viewer = mujoco.viewer.launch_passive(model, data)
        print("\n👁️  VISUALIZANDO EPISODIO 0 - Cierra la ventana para continuar recolección")
    
    episode_data = []
    wind_start_time = time.time()
    
    for step in range(MAX_STEPS):
        # --- A. OBTENER ESTADO ACTUAL ---
        state = get_state(model, data, cart_joint_id, pole_joint_id)
        x, theta = state[0], state[1]
        
        # --- B. APLICAR VIENTO SI ESTÁ HABILITADO ---
        if ep_params['wind']['enabled']:
            t = time.time() - wind_start_time
            wind_force = (ep_params['wind']['amp'] * 
                         np.sin(ep_params['wind']['freq'] * t + ep_params['wind']['phase']))
            data.xfrc_applied[pole_body_id, 0] = wind_force  # Fuerza en eje X del péndulo
        else:
            data.xfrc_applied[pole_body_id, :] = 0.0
        
        # --- C. GENERAR CONTROL EXPLORATORIO (MEZCLA INTELIGENTE) ---
        if np.random.rand() < 0.6:  # 60% exploración pura
            u = np.random.uniform(*CONTROL_RANGE)
        else:  # 40% política energética simple para swing-up
            # Energía deseada para péndulo invertido
            energy_target = 0.5 * 0.1 * 9.81 * 0.5  # m*g*l/2 (ajusta según tu modelo)
            current_energy = 0.5 * 0.1 * (state[3]**2) * (0.5**2) + 0.1 * 9.81 * 0.5 * (1 - np.cos(state[1]))
            u = 80.0 * np.sign(state[3] * np.cos(state[1])) * (energy_target - current_energy)
            u = np.clip(u, CONTROL_RANGE[0], CONTROL_RANGE[1])
        
        data.ctrl[0] = u
        
        # --- D. GUARDAR TRANSICIÓN ANTES DEL STEP ---
        transition = {
            'state': state.copy(),
            'control': float(u),
            'dt': dt,
            # Metadatos para análisis (NO usados en entrenamiento, solo debugging)
            'ep_id': ep,
            'step': step,
            'extra_mass': ep_params['extra_mass'],
            'wind_active': ep_params['wind']['enabled']
        }
        
        # --- E. AVANZAR SIMULACIÓN ---
        mujoco.mj_step(model, data)
        
        # --- F. OBTENER SIGUIENTE ESTADO Y COMPLETAR TRANSICIÓN ---
        next_state = get_state(model, data, cart_joint_id, pole_joint_id)
        transition['next_state'] = next_state.copy()
        
        # --- G. VERIFICAR CONDICIONES DE TERMINACIÓN ---
       
        terminate = (
            abs(next_state[0]) > 2.4 or          # Límite físico del riel (ajusta según tu XML)
            abs(next_state[1]) > 3.0 * np.pi     # Más de 1.5 vueltas (no físico para péndulo simple)
        )
        transition['terminated'] = terminate
        episode_data.append(transition)
        
        # --- H. SINCRONIZAR VISUALIZACIÓN ---
        if viewer is not None:
            viewer.sync()
            time.sleep(dt * 0.5)  # Ligera aceleración para debug
        
        if terminate:
            break
    
    # Cerrar viewer si estaba abierto
    if viewer is not None:
        viewer.close()
    
    # Acumular datos
    dataset.extend(episode_data)
    total_steps += len(episode_data)
    
    # Progreso cada 25 episodios
    if (ep + 1) % 25 == 0:
        elapsed = time.time() - start_time
        steps_per_sec = total_steps / elapsed if elapsed > 0 else 0
        print(f"Episodio {ep+1}/{NUM_EPISODES} | "
              f"Steps este episodio: {len(episode_data)} | "
              f"Total samples: {len(dataset)} | "
              f"Velocidad: {steps_per_sec:.0f} steps/s | "
              f"M_extra: {ep_params['extra_mass']:.3f}kg")

# ==========================================================
# GUARDAR Y REPORTAR
# ==========================================================
np.save(SAVE_PATH, dataset)
print("\n" + "="*60)
print(f"✅ DATASET GENERADO CON ÉXITO")
print(f"📁 Guardado en: {SAVE_PATH}")
print(f"📊 Total de transiciones: {len(dataset)}")
print(f"⏱️  Tiempo total: {time.time() - start_time:.1f}s")

# Estadísticas clave
states = np.array([d['state'] for d in dataset])
controls = np.array([d['control'] for d in dataset])
print(f"\n📈 Rango de estados:")
print(f"   x (posición carrito): [{states[:,0].min():.2f}, {states[:,0].max():.2f}] m")
print(f"   θ (ángulo péndulo):   [{states[:,1].min():.2f}, {states[:,1].max():.2f}] rad")
print(f"   ẋ (vel carrito):      [{states[:,2].min():.2f}, {states[:,2].max():.2f}] m/s")
print(f"   θ̇ (vel angular):      [{states[:,3].min():.2f}, {states[:,3].max():.2f}] rad/s")
print(f"   u (control):          [{controls.min():.1f}, {controls.max():.1f}] N")
print(f"   Masa extra promedio:  {np.mean([d['extra_mass'] for d in dataset]):.3f} kg")
print("="*60)

# Verificación crítica
if len(dataset) < NUM_EPISODES * MAX_STEPS * 0.3:
    print("\n⚠️  ADVERTENCIA: Pocas muestras recolectadas. Verifica:")
    print("   - ¿El péndulo se cae muy rápido? (ajusta reset_to_random_state)")
    print("   - ¿Rango de control demasiado agresivo? (reduce CONTROL_RANGE)")
    print("   - ¿Condiciones de terminación demasiado estrictas?")
else:
    print("\n✨ Dataset con cobertura adecuada del espacio de estados")