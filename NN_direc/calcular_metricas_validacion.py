"""
calcular_metricas_validacion.py — Validación 3: RMSE, MAE, R², AAPE
sobre el set de validación.

Reproduce exactamente el split de validación del entrenamiento original
(misma semilla, mismo dataset, mismo orden que main.py), carga el modelo
guardado y calcula las 4 métricas.

Pegar en NN_direc/ y correr: python calcular_metricas_validacion.py
"""
import numpy as np
import torch

from datos import cargar_datos, preparar_dataset, desnormalizar_y
from control import cargar_modelo

# Misma semilla que main.py, en el mismo punto del flujo.
np.random.seed(42)

X, y, traj_id = cargar_datos()
X_train, y_train, X_val, y_val, stats = preparar_dataset(X, y, traj_id)

modelo, stats_modelo = cargar_modelo("modelo_red_k_F_UCS_ROP_RPM.pth")

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
print("VALIDACIÓN 3 — MÉTRICAS CUANTITATIVAS (set de validación)")
print("=" * 65)
print(f"Puntos de validación: {n}")
print(f"Rango F_roca real en validación: [{y_real.min():.3f}, {y_real.max():.3f}] N")
print()
print(f"RMSE = {rmse:.4f} N   ({100*rmse/rango_y:.3f}% del rango)")
print(f"MAE  = {mae:.4f} N   ({100*mae/rango_y:.3f}% del rango)")
print(f"R²   = {r2:.6f}")

umbral_minimo = 0.05 * rango_y
mask_validos = np.abs(y_real) > umbral_minimo
n_excluidos = n - mask_validos.sum()
aape_todos = np.mean(np.abs(residuo) / np.abs(y_real)) * 100
aape_filtrado = np.mean(np.abs(residuo[mask_validos]) / np.abs(y_real[mask_validos])) * 100

print(f"\nAAPE (todos los puntos):                                 {aape_todos:.3f}%")
print(f"AAPE (excluyendo |F_real|<{umbral_minimo:.2f}N, {n_excluidos} de {n} puntos): {aape_filtrado:.3f}%")