"""
batch_eval_adaptacion.py — Compara DESACTIVADA vs BISECCIÓN en muchos seeds
=============================================================================
Corre N_SEEDS trayectorias distintas, cada una dos veces (con y sin
adaptación de UCS), sin graficar ni imprimir el detalle paso a paso.
Compara dos métricas:
  - error_rms de la trayectoria real (el resultado final que importa)
  - kappa_error_rel_mean (qué tan bien el κ comandado coincide con el κ
    real logrado) — aísla el efecto de conocer UCS del efecto de posibles
    oscilaciones del lazo de control.
"""
import numpy as np
from control import cargar_modelo, simular_perforacion

N_SEEDS = 20   # <<<< AJUSTAR SI CAMBIA

modelo, stats = cargar_modelo("modelo_bha.pth")

filas = []
for seed in range(N_SEEDS):
    m_true  = simular_perforacion(modelo, stats, semilla_trayectoria=seed,
                                   adaptacion_activa=True,  verbose=False, graficar=False)
    m_false = simular_perforacion(modelo, stats, semilla_trayectoria=seed,
                                   adaptacion_activa=False, verbose=False, graficar=False)
    filas.append((seed, m_true, m_false))
    print(f"seed {seed:2d} | True:  err_rms={m_true['error_rms']:.3f}  kappa_err={m_true['kappa_error_rel_mean']*100:5.1f}% "
          f"| False: err_rms={m_false['error_rms']:.3f}  kappa_err={m_false['kappa_error_rel_mean']*100:5.1f}%")

err_rms_true  = np.array([f[1]['error_rms'] for f in filas])
err_rms_false = np.array([f[2]['error_rms'] for f in filas])
kerr_true  = np.array([f[1]['kappa_error_rel_mean'] for f in filas])
kerr_false = np.array([f[2]['kappa_error_rel_mean'] for f in filas])

print("\n" + "=" * 70)
print(f"RESUMEN (n={N_SEEDS} trayectorias)")
print("=" * 70)
print(f"Error RMS trayectoria | True: {err_rms_true.mean():.3f}±{err_rms_true.std():.3f} mm | "
      f"False: {err_rms_false.mean():.3f}±{err_rms_false.std():.3f} mm")
print(f"  -> True (adapta) tiene menor error en {np.mean(err_rms_true < err_rms_false)*100:.0f}% de los seeds")
print(f"Error relativo kappa  | True: {kerr_true.mean()*100:.1f}%±{kerr_true.std()*100:.1f}% | "
      f"False: {kerr_false.mean()*100:.1f}%±{kerr_false.std()*100:.1f}%")
print(f"  -> True (adapta) tiene menor error en {np.mean(kerr_true < kerr_false)*100:.0f}% de los seeds")

try:
    from scipy.stats import wilcoxon
    stat, p = wilcoxon(err_rms_false - err_rms_true)
    print(f"\nWilcoxon pareado (error RMS trayectoria, False-True): p={p:.4f}")
    stat_k, p_k = wilcoxon(kerr_false - kerr_true)
    print(f"Wilcoxon pareado (error relativo kappa, False-True):   p={p_k:.4f}")
except Exception as e:
    print(f"(Wilcoxon no disponible: {e})")