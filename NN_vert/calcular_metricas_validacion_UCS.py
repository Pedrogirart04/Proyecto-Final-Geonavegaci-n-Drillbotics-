"""
calcular_metricas_validacion_UCS.py — Validación 3 (Fase C): RMSE, MAE, R², AAPE
sobre el set de validación.

Pegar en NN_vert/ y correr: python calcular_metricas_validacion_UCS.py
"""
import numpy as np
import torch

from datos import cargar_datos, preparar_dataset, desnormalizar_y
from modelo import RedBHA


def cargar_modelo(ruta="modelo_vert_WOB_RPM_ROP_UCS.pth"):
    checkpoint = torch.load(ruta, weights_only=False)
    modelo = RedBHA(capas=checkpoint['capas'])
    modelo.load_state_dict(checkpoint['model_state_dict'])
    modelo.eval()
    return modelo, checkpoint['stats']


np.random.seed(42)
X, y, probeta_id = cargar_datos()
X_train, y_train, X_val, y_val, stats = preparar_dataset(X, y, probeta_id)

modelo, stats_modelo = cargar_modelo()

if not (np.allclose(stats['X_min'], stats_modelo['X_min']) and
        np.allclose(stats['X_max'], stats_modelo['X_max'])):
    print("¡ATENCIÓN! El dataset actual no coincide con el usado para entrenar el modelo guardado.")

modelo.eval()
with torch.no_grad():
    y_pred_norm = modelo(X_val)

y_pred = desnormalizar_y(y_pred_norm, stats_modelo).numpy()[:, 0]
y_real = desnormalizar_y(y_val, stats_modelo).numpy()[:, 0]

residuo = y_real - y_pred
n = len(y_real)

rmse = np.sqrt(np.mean(residuo**2))
mae  = np.mean(np.abs(residuo))
ss_res = np.sum(residuo**2)
ss_tot = np.sum((y_real - y_real.mean())**2)
r2 = 1 - ss_res / ss_tot
rango_y = y_real.max() - y_real.min()

print("=" * 65)
print("VALIDACIÓN 3 (Fase C) — MÉTRICAS CUANTITATIVAS (set de validación)")
print("=" * 65)
print(f"Puntos de validación: {n}")
print(f"Rango UCS real en validación: [{y_real.min():.3f}, {y_real.max():.3f}] MPa")
print()
print(f"RMSE = {rmse:.4f} MPa   ({100*rmse/rango_y:.3f}% del rango)")
print(f"MAE  = {mae:.4f} MPa   ({100*mae/rango_y:.3f}% del rango)")
print(f"R²   = {r2:.6f}")

umbral_minimo = 0.05 * rango_y
mask_validos = np.abs(y_real) > umbral_minimo
n_excluidos = n - mask_validos.sum()
aape_todos = np.mean(np.abs(residuo) / np.abs(y_real)) * 100

print(f"\nAAPE: {aape_todos:.4f}%  (puntos excluidos por UCS cercano a 0: {n_excluidos})")