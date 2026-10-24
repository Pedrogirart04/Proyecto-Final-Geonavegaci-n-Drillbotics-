"""
sweep_error_saturacion.py — Confirmación True vs False con ruido de IMU
=============================================================================
Esta versión va directo a confirmar si
la adaptación de UCS (ADAPTACION_ACTIVA) sigue ayudando cuando hay ruido de
IMU y de UCS local realista (seteado en control.py: RUIDO_IMU_I, RUIDO_IMU_A,
RUIDO_UCS_LOCAL), que es la pregunta que estaba abierta.
"""
import numpy as np
from control import cargar_modelo, simular_perforacion

mejor = 0.75  # [mm] Ya encontrado en el barrido anterior, sin ruido

modelo, stats = cargar_modelo("modelo_red_k_F_UCS_ROP_RPM.pth")

print(f"Confirmando con ERROR_SATURACION={mejor} (con ruido de IMU/UCS activado): True vs False")
N_SEEDS_CONFIRMACION = 200
err_true  = np.array([simular_perforacion(modelo, stats, semilla_trayectoria=s, adaptacion_activa=True,  error_saturacion=mejor, verbose=False, graficar=False)['error_rms'] for s in range(N_SEEDS_CONFIRMACION)])
err_false = np.array([simular_perforacion(modelo, stats, semilla_trayectoria=s, adaptacion_activa=False, error_saturacion=mejor, verbose=False, graficar=False)['error_rms'] for s in range(N_SEEDS_CONFIRMACION)])
print(f"  (n={N_SEEDS_CONFIRMACION} seeds)")
print(f"  True:  {err_true.mean():.3f}±{err_true.std():.3f} mm")
print(f"  False: {err_false.mean():.3f}±{err_false.std():.3f} mm")
print(f"  True mejor en {np.mean(err_true < err_false)*100:.0f}% de los seeds")

try:
    from scipy.stats import wilcoxon
    stat, p = wilcoxon(err_false - err_true, alternative='greater')
    print(f"  Wilcoxon unilateral (H1: True < False): p={p:.4f}")
except Exception as e:
    print(f"  (Wilcoxon no disponible: {e})")