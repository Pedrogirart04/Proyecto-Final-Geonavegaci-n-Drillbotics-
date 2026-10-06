"""
sweep_error_saturacion.py — Busca el mejor ERROR_SATURACION
=============================================================================
Prueba varios valores candidatos, cada uno con N_SEEDS trayectorias y
adaptación activa (el modo real a usar), y se queda con el de menor error
RMS promedio. Al final confirma que con ese valor la adaptación sigue
ganando con margen frente a desactivada.
"""
import numpy as np
from control import cargar_modelo, simular_perforacion

N_SEEDS = 20   # <<<< AJUSTAR SI CAMBIA
VALORES_A_PROBAR = [0.55, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0]   # [mm] <<<< AJUSTAR SI CAMBIA

modelo, stats = cargar_modelo("modelo_bha.pth")

print("Barriendo ERROR_SATURACION (adaptación activa)...")
resultados = {}
for es in VALORES_A_PROBAR:
    errores = np.array([
        simular_perforacion(modelo, stats, semilla_trayectoria=seed,
                             adaptacion_activa=True, error_saturacion=es,
                             verbose=False, graficar=False)['error_rms']
        for seed in range(N_SEEDS)
    ])
    resultados[es] = errores
    print(f"  ERROR_SATURACION={es:4.1f} mm | error_rms: {errores.mean():.3f}±{errores.std():.3f} mm")

mejor = min(resultados, key=lambda k: resultados[k].mean())
print(f"\nMejor valor: ERROR_SATURACION={mejor} mm (error_rms medio={resultados[mejor].mean():.3f} mm)")

print(f"\nConfirmando con ERROR_SATURACION={mejor}: True vs False")
N_SEEDS_CONFIRMACION = 60   # <<<< más seeds que en el barrido, para una lectura más firme
err_true  = np.array([simular_perforacion(modelo, stats, semilla_trayectoria=s, adaptacion_activa=True,  error_saturacion=mejor, verbose=False, graficar=False)['error_rms'] for s in range(N_SEEDS_CONFIRMACION)])
err_false = np.array([simular_perforacion(modelo, stats, semilla_trayectoria=s, adaptacion_activa=False, error_saturacion=mejor, verbose=False, graficar=False)['error_rms'] for s in range(N_SEEDS_CONFIRMACION)])
print(f"  (n={N_SEEDS_CONFIRMACION} seeds)")
print(f"  True:  {err_true.mean():.3f}±{err_true.std():.3f} mm")
print(f"  False: {err_false.mean():.3f}±{err_false.std():.3f} mm")
print(f"  True mejor en {np.mean(err_true < err_false)*100:.0f}% de los seeds")

try:
    from scipy.stats import wilcoxon
    # Unilateral: la hipótesis es específicamente que True (adapta) tiene
    # MENOR error que False, no solo "distinto" -- ya lo justificamos con
    # la física (evaluar_red_roca depende fuerte de epsilon).
    stat, p = wilcoxon(err_false - err_true, alternative='greater')
    print(f"  Wilcoxon unilateral (H1: True < False): p={p:.4f}")
except Exception as e:
    print(f"  (Wilcoxon no disponible: {e})")