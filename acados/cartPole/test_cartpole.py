"""
test_acados_cartpole.py
Script para probar que el OCP de Acados compila correctamente
"""

import numpy as np
from acados_template import AcadosOcpSolver
import sys

# Importar tus módulos
try:
    from model_cartpole import model_cartpole
    from ocp_cartpole import create_ocp
    print("✅ Módulos importados correctamente")
except ImportError as e:
    print(f"❌ Error al importar módulos: {e}")
    sys.exit(1)

def test_model():
    """Prueba que el modelo se crea correctamente"""
    print("\n" + "="*70)
    print("TEST 1: Creando modelo del cartpole")
    print("="*70)
    
    try:
        model = model_cartpole()
        print(f"✅ Modelo creado: {model.name}")
        print(f"   Estados (nx): {model.x.size()[0]}")
        print(f"   Controles (nu): {model.u.size()[0]}")
        print(f"   Parámetros (np): {model.p.size()[0]}")
        return True
    except Exception as e:
        print(f"❌ Error creando modelo: {e}")
        return False

def test_ocp():
    """Prueba que el OCP se crea correctamente"""
    print("\n" + "="*70)
    print("TEST 2: Creando OCP")
    print("="*70)
    
    try:
        ocp = create_ocp()
        print(f"✅ OCP creado correctamente")
        print(f"   Modelo: {ocp.model.name}")
        print(f"   Horizonte: {ocp.solver_options.tf} s")
        print(f"   N steps: {ocp.solver_options.N_horizon}")
        print(f"   nx: {ocp.dims.nx}")
        print(f"   nu: {ocp.dims.nu}")
        print(f"   np: {ocp.dims.np}")
        print(f"   ny: {ocp.dims.ny}")
        print(f"   ny_e: {ocp.dims.ny_e}")
        print(f"   Integrador: {ocp.solver_options.integrator_type}")
        print(f"   Solver: {ocp.solver_options.nlp_solver_type}")
        return ocp
    except Exception as e:
        print(f"❌ Error creando OCP: {e}")
        import traceback
        traceback.print_exc()
        return None
    
def test_solver(ocp):
    """Prueba que el solver de Acados compila"""
    print("\n" + "="*70)
    print("TEST 3: Compilando solver de Acados")
    print("="*70)
    print("⏳ Esto puede tardar un momento...")
    
    try:
        solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
        print("✅ Solver compilado exitosamente!")
        return solver
    except Exception as e:
        print(f"❌ Error compilando solver: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_solve(solver, ocp):
    """Prueba resolver un problema simple"""
    print("\n" + "="*70)
    print("TEST 4: Resolviendo OCP de prueba")
    print("="*70)
    
    try:
        # Condición inicial: péndulo casi vertical con pequeña perturbación
        x0 = np.array([0.0, 0.1, 0.0, 0.0])  # [x, theta, x_dot, theta_dot]
        
        # Setear condición inicial
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)
        
        # Resolver
        status = solver.solve()
        
        if status == 0:
            print("✅ Solver convergió exitosamente!")
            
            # Obtener solución
            u0 = solver.get(0, "u")
            x1 = solver.get(1, "x")
            
            print(f"\n📊 Resultados:")
            print(f"   Estado inicial: x={x0[0]:.3f}, θ={np.rad2deg(x0[1]):.1f}°, "
                  f"ẋ={x0[2]:.3f}, θ̇={x0[3]:.3f}")
            print(f"   Control óptimo: F={u0[0]:.2f} N")
            print(f"   Próximo estado: x={x1[0]:.3f}, θ={np.rad2deg(x1[1]):.1f}°")
            
            # Verificar costo
            cost = 0.0
            for i in range(ocp.dims.N):
                cost += solver.get_cost()
            print(f"   Costo total: {cost:.4f}")
            
            return True
        else:
            print(f"⚠️  Solver no convergió. Status: {status}")
            return False
            
    except Exception as e:
        print(f"❌ Error resolviendo OCP: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_multiple_scenarios(solver, ocp):
    """Prueba varios escenarios iniciales"""
    print("\n" + "="*70)
    print("TEST 5: Probando múltiples escenarios")
    print("="*70)
    
    scenarios = [
        ("Péndulo vertical", np.array([0.0, 0.0, 0.0, 0.0])),
        ("Perturbación pequeña", np.array([0.0, 0.1, 0.0, 0.0])),
        ("Cart desplazado", np.array([0.2, 0.0, 0.0, 0.0])),
        ("Velocidad inicial", np.array([0.0, 0.0, 0.5, 0.0])),
    ]
    
    results = []
    
    for name, x0 in scenarios:
        try:
            solver.set(0, "lbx", x0)
            solver.set(0, "ubx", x0)
            status = solver.solve()
            
            if status == 0:
                u0 = solver.get(0, "u")
                results.append((name, True, u0[0]))
                print(f"✅ {name:25s}: F = {u0[0]:7.2f} N")
            else:
                results.append((name, False, 0.0))
                print(f"⚠️  {name:25s}: No convergió")
                
        except Exception as e:
            results.append((name, False, 0.0))
            print(f"❌ {name:25s}: Error - {e}")
    
    success_count = sum(1 for _, success, _ in results if success)
    print(f"\n📊 Resultados: {success_count}/{len(scenarios)} escenarios exitosos")
    
    return success_count == len(scenarios)

def main():
    """Ejecuta todos los tests"""
    print("\n" + "="*70)
    print("🧪 PRUEBAS DE COMPILACIÓN - ACADOS CARTPOLE")
    print("="*70)
    
    all_passed = True
    
    # Test 1: Modelo
    if not test_model():
        print("\n❌ FALLO: El modelo no se pudo crear")
        return False
    
    # Test 2: OCP
    ocp = test_ocp()
    if ocp is None:
        print("\n❌ FALLO: El OCP no se pudo crear")
        return False
    
    # Test 3: Compilación del solver
    solver = test_solver(ocp)
    if solver is None:
        print("\n❌ FALLO: El solver no compiló")
        return False
    
    # Test 4: Resolver un problema
    if not test_solve(solver, ocp):
        print("\n⚠️  ADVERTENCIA: El solver no pudo resolver el problema de prueba")
        all_passed = False
    
    # Test 5: Múltiples escenarios
    if not test_multiple_scenarios(solver, ocp):
        print("\n⚠️  ADVERTENCIA: Algunos escenarios fallaron")
        all_passed = False
    
    # Resumen final
    print("\n" + "="*70)
    if all_passed:
        print("✅ TODOS LOS TESTS PASARON")
        print("="*70)
        print("🎉 Tu configuración de Acados está lista para usar!")
        print("\n💡 Próximo paso: Integrar con MuJoCo para control en tiempo real")
    else:
        print("⚠️  TESTS COMPLETADOS CON ADVERTENCIAS")
        print("="*70)
        print("El solver compila pero algunos tests fallaron.")
        print("Revisa los mensajes de error arriba.")
    
    return all_passed

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)