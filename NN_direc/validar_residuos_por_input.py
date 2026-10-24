"""
validar_residuos_por_input.py — Validación 4: residuos por input
===================================================================
Agrupa el set de validación en bins según el valor de cada input
(kappa, ROP, RPM, UCS) y mira si el error (residuo = F_real - F_pred)
se concentra en alguna zona del rango, en vez de estar repartido
uniformemente. Usa el mismo split reproducible que la Validación 3.

Pegar en NN_direc/ y correr: python validar_residuos_por_input.py
"""
import numpy as np
import torch
import matplotlib.pyplot as plt

from datos import cargar_datos
from control import cargar_modelo

NOMBRES = ['kappa [1/mm]', 'ROP [mm/min]', 'RPM', 'UCS/epsilon [MPa]']
N_BINS = 10

np.random.seed(42)
X, y, traj_id = cargar_datos()

X_min = X.min(axis=0); X_max = X.max(axis=0)
y_min = y.min(axis=0); y_max = y.max(axis=0)
X_norm = (X - X_min) / (X_max - X_min + 1e-8)

ids_unicos = np.unique(traj_id)
n_val_traj = max(1, int(len(ids_unicos) * 0.2))
ids_perm = np.random.permutation(ids_unicos)
ids_val = set(ids_perm[:n_val_traj].tolist())
mask_val = np.array([tid in ids_val for tid in traj_id])

X_val_norm = torch.tensor(X_norm[mask_val], dtype=torch.float32)
X_val_real = X[mask_val]
y_real = y[mask_val, 0]

modelo, stats_modelo = cargar_modelo("modelo_red_k_F_UCS_ROP_RPM.pth")
modelo.eval()
with torch.no_grad():
    y_pred_norm = modelo(X_val_norm).numpy()[:, 0]

y_pred = y_pred_norm * (stats_modelo['y_max'][0] - stats_modelo['y_min'][0]) + stats_modelo['y_min'][0]
residuo = y_real - y_pred

for i, nombre in enumerate(NOMBRES):
    valores = X_val_real[:, i]
    bins = np.linspace(valores.min(), valores.max(), N_BINS + 1)
    bin_idx = np.clip(np.digitize(valores, bins[1:-1]), 0, N_BINS - 1)

    print(f"\n{nombre}:")
    print(f"{'rango bin':>22} {'n':>6} {'MAE':>9} {'RMSE':>9} {'res.medio':>10}")
    for b in range(N_BINS):
        sel = bin_idx == b
        if sel.sum() == 0:
            continue
        r = residuo[sel]
        rango_bin = f"[{bins[b]:.4f},{bins[b+1]:.4f}]"
        print(f"{rango_bin:>22} {sel.sum():>6} {np.mean(np.abs(r)):>9.4f} "
              f"{np.sqrt(np.mean(r**2)):>9.4f} {r.mean():>10.4f}")

    # Gráfico: residuo (con signo) vs valor del input
    plt.figure(figsize=(7, 4))
    plt.scatter(valores, residuo, s=3, alpha=0.3)
    plt.axhline(0, color='r', linestyle='--')
    plt.xlabel(nombre)
    plt.ylabel("Residuo (F_real - F_pred) [N]")
    plt.title(f"Residuo vs {nombre}")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"residuo_vs_input_{i}.png", dpi=120)
    plt.close()

print("\nGráficos guardados: residuo_vs_input_0..3.png")