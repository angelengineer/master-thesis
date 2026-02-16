import  numpy as np
# En tu loop de simulación (versión de recolección de datos)
dataset = []

for episode in range(100):
    # Reset con parámetros aleatorios
    m_cart = 1.0 + np.random.uniform(-0.3, 0.3)   # ±30% masa carrito
    m_pole = 0.1 + np.random.uniform(-0.03, 0.03) # ±30% masa péndulo
    
    # Aplicar control aleatorio/exploratorio
    for step in range(500):
        u = np.random.uniform(-50, 50)  # exploración activa
        x_current = get_state(data)
        
        mujoco.mj_step(model, data)
        x_next = get_state(data)
        
        # Guardar transición
        dataset.append({
            'x': x_current,
            'u': u,
            'x_next': x_next,
            'dt': dt
        })