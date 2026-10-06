"""
main.py — Ejecución principal (Fase C — Red de estimación de UCS)
=====================================================================
Orquesta el entrenamiento de la red inversa (ROP, RPM, WOB) -> UCS:
  1. Carga el dataset (sintético por ahora, real más adelante — MISMO
     código, solo cambia config.DATASET_PATH)
  2. Prepara y normaliza el dataset (split agrupado por probeta)
  3. Crea la red neuronal (arquitectura chica, ver config.py)
  4. Entrena (Adam + L-BFGS)
  5. Evalúa con gráficos y métricas
  6. Guarda el modelo entrenado
  7. Ejemplo rápido de uso: estimar UCS a partir de (ROP, RPM, WOB)

NO integra todavía con el loop de control direccional (control.py /
navegacion.py) — eso es un paso posterior, a propósito (ver fase-c-plan.md).

Para correr: python main.py
"""

import torch
import numpy as np
import time

from config import ARQUITECTURA
from datos import cargar_datos, preparar_dataset, normalizar_X, desnormalizar_y
from modelo import RedBHA
from entrenamiento import entrenar, evaluar


if __name__ == "__main__":

    torch.manual_seed(42)
    np.random.seed(42)

    # ── 1. Cargar datos ──────────────────────────────────────
    t0 = time.time()
    print("=" * 65)
    print("CARGANDO DATOS (Fase C — UCS)")
    print("=" * 65)
    X, y, probeta_id = cargar_datos()
    print(f"Tiempo carga datos: {time.time() - t0:.2f}s")

    # ── 2. Preparar dataset ───────────────────────────────────
    t0 = time.time()
    X_train, y_train, X_val, y_val, stats = preparar_dataset(X, y, probeta_id)
    print(f"Tiempo preparación dataset: {time.time() - t0:.2f}s")

    # ── 3. Crear la red ───────────────────────────────────────
    print("\n" + "=" * 65)
    print("ENTRENAMIENTO")
    print("=" * 65)
    modelo = RedBHA()

    # ── 4. Entrenar ───────────────────────────────────────────
    t0 = time.time()
    historial_train, historial_val = entrenar(
        modelo, X_train, y_train, X_val, y_val
    )
    print(f"Tiempo entrenamiento: {time.time() - t0:.2f}s")

    # ── 5. Guardar modelo ─────────────────────────────────────
    torch.save({
        'model_state_dict': modelo.state_dict(),
        'stats': stats,
        'capas': ARQUITECTURA
    }, "modelo_vert_WOB_RPM_ROP_UCS.pth")
    print("\nModelo guardado en: modelo_vert_WOB_RPM_ROP_UCS.pth")

    # ── 6. Evaluar ────────────────────────────────────────────
    t0 = time.time()
    print("\n" + "=" * 65)
    print("EVALUACIÓN")
    print("=" * 65)
    evaluar(modelo, X_val, y_val, stats, historial_train, historial_val)
    print(f"Tiempo evaluación: {time.time() - t0:.2f}s")

        # ── 7. Ejemplo de uso: estimar UCS a partir de (ROP, RPM, WOB) ──
    print("\n" + "=" * 65)
    print("EJEMPLO — ESTIMACIÓN DE UCS")
    print("=" * 65)
    modelo.eval()
    rop_ejemplo = 15.0
    rpm_ejemplo = 1500.0
    UCS_real_ejemplo = 25.0  # [MPa] valor "verdadero" supuesto, DENTRO del rango de entrenamiento
    a = 1.5 * 25.4 / 2
    wob_ejemplo = np.pi * a * UCS_real_ejemplo * rop_ejemplo / (2 * rpm_ejemplo)  # WOB que daría la fórmula

    X_ejemplo = np.array([[rop_ejemplo, rpm_ejemplo, wob_ejemplo]])
    X_ejemplo_norm = normalizar_X(X_ejemplo, stats)
    with torch.no_grad():
        UCS_estimado = desnormalizar_y(modelo(X_ejemplo_norm), stats).item()

    print(f"  ROP = {rop_ejemplo} mm/min, RPM = {rpm_ejemplo}, WOB = {wob_ejemplo:.2f} N (calculado con UCS={UCS_real_ejemplo} MPa)")
    print(f"  UCS estimado por la red: {UCS_estimado:.2f} MPa (real: {UCS_real_ejemplo} MPa)")