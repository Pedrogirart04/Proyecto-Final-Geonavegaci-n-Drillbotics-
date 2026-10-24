"""
validar_residuos_por_input_UCS.py — Validación 4 (Fase C): residuos por input
================================================================================
Pegar en NN_vert/ y correr: python validar_residuos_por_input_UCS.py
"""
import numpy as np
import torch

from datos import cargar_datos, desnormalizar_y
from modelo import RedBHA

NOMBRES = ['ROP [mm/min]', 'RPM', 'WOB [N]']
N_BINS = 8  # menos bins que Fase B porque el set de validación es más chico (200 puntos)


def cargar_modelo(ruta="modelo_vert_WOB_RPM_ROP_UCS.pth"):
    checkpoint = torch.load(ruta, weights_only=False)
    modelo = RedBHA(capas=checkpoint['capas'])
    modelo.load_state_dict(checkpoint['model_state_dict'])
    modelo.eval()
    return modelo, checkpoint['stats']


np.random.seed(42)
X, y, probeta_id = cargar_datos()

X_min = X.min(axis=0); X_max = X.max(axis=0)
X_norm = (X - X_min) / (X_max - X_min + 1e-8)

ids_unicos = np.unique(probeta_id)
n_val = max(1, int(len(ids_unicos) * 0.2))
ids_perm = np.random.permutation(ids_unicos)
ids_val = set(ids_perm[:n_val].tolist())
mask_val = np.array([pid in ids_val for pid in probeta_id])

X_val_norm = torch.tensor(X_norm[mask_val], dtype=torch.float32)
X_val_real = X[mask_val]
y_real = y[mask_val, 0]

modelo, stats_modelo = cargar_modelo()
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
    print(f"{'rango bin':>20} {'n':>5} {'MAE':>8} {'RMSE':>8} {'res.medio':>10}")
    for b in range(N_BINS):
        sel = bin_idx == b
        if sel.sum() == 0:
            continue
        r = residuo[sel]
        rango_bin = f"[{bins[b]:.2f},{bins[b+1]:.2f}]"
        print(f"{rango_bin:>20} {sel.sum():>5} {np.mean(np.abs(r)):>8.4f} "
              f"{np.sqrt(np.mean(r**2)):>8.4f} {r.mean():>10.4f}")