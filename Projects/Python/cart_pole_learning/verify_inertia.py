"""
Script para verificar las inercias actuales en tu modelo de MuJoCo
Ejecuta esto para ver qué valores calculó MuJoCo automáticamente
"""

import mujoco
import numpy as np

# Cargar modelo
model = mujoco.MjModel.from_xml_path("cart_pole_description/cartpole.xml")

print("=" * 70)
print("VERIFICACIÓN DE INERCIAS EN EL MODELO MUJOCO")
print("=" * 70)

# IDs de los bodies
try:
    cart_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cart_link")
    pole_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pole_link")
    
    print("\n🔹 CART (Carro):")
    print(f"   Masa: {model.body_mass[cart_body_id]:.4f} kg")
    print(f"   Inercia diagonal (Ixx, Iyy, Izz): {model.body_inertia[cart_body_id]}")
    print(f"   Centro de masa (pos): {model.body_ipos[cart_body_id]}")
    
    print("\n🔹 POLE (Péndulo):")
    print(f"   Masa: {model.body_mass[pole_body_id]:.4f} kg")
    print(f"   Inercia diagonal (Ixx, Iyy, Izz): {model.body_inertia[pole_body_id]}")
    print(f"   Centro de masa (pos): {model.body_ipos[pole_body_id]}")
    
    # Calcular inercia esperada para una barra
    m_pole = model.body_mass[pole_body_id]
    com_pole = model.body_ipos[pole_body_id]
    L_pole = abs(com_pole[2]) * 2  # Distancia al COM × 2
    
    I_expected = (1/12) * m_pole * L_pole**2
    I_actual = model.body_inertia[pole_body_id][0]  # Ixx
    
    print("\n📊 ANÁLISIS DEL POLE:")
    print(f"   Longitud estimada: {L_pole:.4f} m")
    print(f"   Inercia esperada (barra delgada): {I_expected:.6f} kg·m²")
    print(f"   Inercia calculada por MuJoCo: {I_actual:.6f} kg·m²")
    
    if abs(I_expected - I_actual) / I_expected < 0.1:
        print("   ✅ Las inercias coinciden (< 10% diferencia)")
    else:
        diff_pct = abs(I_expected - I_actual) / I_expected * 100
        print(f"   ⚠️  Diferencia significativa: {diff_pct:.1f}%")
        print("   → MuJoCo calculó inercias del mesh, pueden no ser correctas")
        print("   → Recomendación: especifica <inertial> explícitamente")
    
except Exception as e:
    print(f"\n❌ Error al acceder a los bodies: {e}")
    print("   Verifica que los nombres 'cart_link' y 'pole_link' sean correctos")

# Información adicional útil
print("\n" + "=" * 70)
print("INFORMACIÓN ADICIONAL DEL MODELO")
print("=" * 70)

print(f"\nNúmero de bodies: {model.nbody}")
print(f"Número de joints: {model.njnt}")
print(f"Número de actuadores: {model.nu}")
print(f"Timestep: {model.opt.timestep} s")

# Listar todos los bodies
print("\nBodies en el modelo:")
for i in range(model.nbody):
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if body_name:  # El world body no tiene nombre
        print(f"  [{i}] {body_name}")

# Listar todos los joints
print("\nJoints en el modelo:")
for i in range(model.njnt):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(f"  [{i}] {joint_name}")

print("\n" + "=" * 70)
print("\n💡 SIGUIENTE PASO:")
print("   1. Si las inercias son muy diferentes a las esperadas:")
print("      → Añade <inertial> explícitamente en el XML")
print("   2. Si las inercias son correctas:")
print("      → Verifica que tu OCP use estos mismos valores")
print("   3. Para máxima precisión:")
print("      → Usa los valores de tu modelo Matlab/Simulink")
print("=" * 70 + "\n")