import mujoco
import mujoco.viewer
import numpy as np
import time
from scipy.linalg import solve_continuous_are

# Cargar modelo
model = mujoco.MjModel.from_xml_path("../../Fusion/cartPole_description/mujoco/cartpole.xml")
data = mujoco.MjData(model)

# Variable global para la posición deseada del cart
x_desired = 0.0

def get_lqr_gains(model, data):
    """Calcula ganancias LQR para el péndulo invertido"""
    
    # Configurar punto de equilibrio (péndulo arriba)
    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0.0      # Cart en el centro
    data.qpos[1] = 0.0      # Pole arriba (0° en tu modelo)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    
    mujoco.mj_forward(model, data)
    
    nv = model.nv  # 2 (cart + pole)
    nu = model.nu  # 1 (motor)
    
    # Calcular derivadas de la dinámica
    eps = 1e-6
    
    # Guardar estado nominal
    qpos0 = data.qpos.copy()
    qvel0 = data.qvel.copy()
    ctrl0 = data.ctrl.copy()
    mujoco.mj_forward(model, data)
    qacc0 = data.qacc.copy()
    
    # df/dq
    DfDq = np.zeros((nv, nv))
    for i in range(nv):
        data.qpos[:] = qpos0
        data.qvel[:] = qvel0
        data.ctrl[:] = ctrl0
        data.qpos[i] += eps
        mujoco.mj_forward(model, data)
        DfDq[:, i] = (data.qacc - qacc0) / eps
    
    # df/dv
    DfDv = np.zeros((nv, nv))
    for i in range(nv):
        data.qpos[:] = qpos0
        data.qvel[:] = qvel0
        data.ctrl[:] = ctrl0
        data.qvel[i] += eps
        mujoco.mj_forward(model, data)
        DfDv[:, i] = (data.qacc - qacc0) / eps
    
    # df/du
    DfDu = np.zeros((nv, nu))
    for i in range(nu):
        data.qpos[:] = qpos0
        data.qvel[:] = qvel0
        data.ctrl[:] = ctrl0
        data.ctrl[i] += eps
        mujoco.mj_forward(model, data)
        DfDu[:, i] = (data.qacc - qacc0) / eps
    
    # Construir matrices de estado continuo
    A = np.zeros((2*nv, 2*nv))
    B = np.zeros((2*nv, nu))
    
    A[0:nv, nv:2*nv] = np.eye(nv)
    A[nv:2*nv, 0:nv] = DfDq
    A[nv:2*nv, nv:2*nv] = DfDv
    
    B[nv:2*nv, :] = DfDu
    
    print("Matriz A (dinámica continua):")
    print(A)
    print("\nMatriz B (control):")
    print(B.flatten())
    
    # Verificar autovalores
    eigvals_A = np.linalg.eigvals(A)
    print(f"\nAutovalores de A: {eigvals_A}")
    unstable = np.any(np.real(eigvals_A) > 1e-6)
    print(f"Sistema inestable (esperado): {unstable}")
    
    # Diseño LQR
    Q = np.diag([
        100.0,   # x_cart
        1000.0,  # θ
        1.0,     # ẋ_cart
        10.0     # θ̇
    ])
    
    R = np.array([[0.1]])
    
    # Resolver ecuación de Riccati
    try:
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        
        print(f"\nGanancias K: {K.flatten()}")
        
        # Verificar estabilidad en lazo cerrado
        A_closed = A - B @ K
        eigvals_closed = np.linalg.eigvals(A_closed)
        print(f"Autovalores en lazo cerrado: {eigvals_closed}")
        stable = np.all(np.real(eigvals_closed) < -1e-6)
        print(f"Sistema estable en lazo cerrado: {stable}")
        
        return K
        
    except np.linalg.LinAlgError as e:
        print(f"❌ Error resolviendo ARE: {e}")
        return None

# Calcular ganancias
print("="*70)
print("CALCULANDO CONTROL LQR")
print("="*70)
K = get_lqr_gains(model, data)

if K is None:
    print("\n❌ No se pudo calcular LQR")
    exit()

print("\n✅ LQR calculado exitosamente")
print("="*70)
print("\n🎮 CONTROLES (usa las teclas en la ventana de MuJoCo):")
print("  A  - Mover cart a la IZQUIERDA")
print("  D  - Mover cart a la DERECHA")
print("  S  - Volver al CENTRO")
print("  R  - RESETEAR sistema")
print("  ESC - Salir")
print("="*70)

# Variables de estado del teclado
keys_pressed = {
    'a': False,
    'd': False,
    's': False,
    'r': False
}
last_key_time = {'s': 0, 'r': 0}  # Para debounce

# Callback de teclado
def keyboard_callback(keycode):
    global x_desired, keys_pressed, last_key_time
    
    current_time = time.time()
    
    # Detectar tecla presionada
    if keycode == ord('A') or keycode == ord('a'):
        keys_pressed['a'] = True
    elif keycode == ord('D') or keycode == ord('d'):
        keys_pressed['d'] = True
    elif keycode == ord('S') or keycode == ord('s'):
        # Debounce para evitar múltiples activaciones
        if current_time - last_key_time['s'] > 0.3:
            x_desired = 0.0
            print(f"🎯 CENTRADO | x_deseada = {x_desired:.3f}m")
            last_key_time['s'] = current_time
    elif keycode == ord('R') or keycode == ord('r'):
        if current_time - last_key_time['r'] > 0.3:
            mujoco.mj_resetData(model, data)
            data.qpos[0] = 0.0
            data.qpos[1] = 0.1
            data.qvel[:] = 0.0
            x_desired = 0.0
            print(f"🔄 SISTEMA RESETEADO")
            last_key_time['r'] = current_time

# Simulación
print("\n🚀 Iniciando simulación...")
print("👉 Haz clic en la ventana de MuJoCo y usa las teclas A/D para mover\n")

with mujoco.viewer.launch_passive(model, data, key_callback=keyboard_callback) as viewer:
    # Reset inicial
    mujoco.mj_resetData(model, data)
    data.qpos[0] = 0.0
    data.qpos[1] = 0.1  # 5.7° de perturbación
    data.qvel[:] = 0.0
    
    step = 0
    last_print_time = time.time()
    
    while viewer.is_running():
        step_start = time.time()
        
        # Actualizar posición deseada basado en teclas presionadas
        velocity = 0.1  # Velocidad de movimiento
        
        if keys_pressed['a']:
            x_desired -= velocity
            x_desired = max(x_desired, -0.40)
        
        if keys_pressed['d']:
            x_desired += velocity
            x_desired = min(x_desired, 0.40)
        
        # Reset flags (las teclas se mantienen presionadas hasta que se suelten)
        # En este caso, asumimos que se liberan cada frame para control incremental
        keys_pressed['a'] = False
        keys_pressed['d'] = False
        
        # Leer estado actual
        x_cart = data.qpos[0]
        theta_pole = data.qpos[1]
        v_cart = data.qvel[0]
        omega_pole = data.qvel[1]
        
        # Estado: [x, θ, ẋ, θ̇]
        state = np.array([x_cart, theta_pole, v_cart, omega_pole])
        
        # Referencia (equilibrio modificable)
        state_ref = np.array([x_desired, 0.0, 0.0, 0.0])
        
        # Error
        error = state - state_ref
        
        # Ley de control: u = -K*(x - x_ref)
        u = -K @ error
        
        # Saturar control
        u = np.clip(u, -100, 100)
        
        # Aplicar
        data.ctrl[0] = u[0]
        
        # Simular
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Mostrar info cada 1 segundo
        if time.time() - last_print_time > 1.0:
            theta_deg = np.rad2deg(theta_pole)
            print(f"t={data.time:5.2f}s | x={x_cart:6.3f}m → {x_desired:6.3f}m | "
                  f"θ={theta_deg:6.1f}° | u={u[0]:7.2f}N")
            last_print_time = time.time()
        
        step += 1
        
        # Control de tiempo real
        elapsed = time.time() - step_start
        sleep_time = model.opt.timestep - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)