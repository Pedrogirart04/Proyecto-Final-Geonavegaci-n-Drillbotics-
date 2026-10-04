"""
main.py — Ejecución principal
==============================
Orquesta el entrenamiento de la red:
  1. Carga el dataset real (generado en MATLAB con el modelo de Perneder)
  2. Prepara y normaliza el dataset
  3. Crea la red neuronal
  4. Entrena (Adam + L-BFGS)
  5. Ejemplo de uso del controlador completo (red + descomposición en pads)
  6. Evalúa con gráficos
  7. Guarda el modelo entrenado

Para correr: python main.py
"""

import torch
import numpy as np
import time
import matplotlib.pyplot as plt


from config import ARQUITECTURA
from datos import cargar_datos, preparar_dataset
from modelo import RedBHA
from entrenamiento import entrenar, evaluar
from optimizador import control_completo


if __name__ == "__main__":
    
    # Entrenar desde cero
    # ── 0. Configuracion de todo ──────────────────────────────────────
    # Archivo config.py

    # Semillas para reproducibilidad
    torch.manual_seed(42)
    np.random.seed(42)

    # ── 1. Cargar datos ──────────────────────────────────────
    t0 = time.time()
    print("=" * 65)
    print("CARGANDO DATOS")
    print("=" * 65)
    X, y, traj_id = cargar_datos()
    
    # ── Validación de distribución del dataset ────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle("Distribución del Dataset", fontsize=14)

    axes[0].hist(X[:, 0], bins=50, color='steelblue', edgecolor='white')
    axes[0].set_xlabel("kappa [1/mm]")
    axes[0].set_ylabel("Cantidad de puntos")
    axes[0].set_title("Distribución de curvatura (input)")
    axes[0].grid(True)

    axes[1].hist(y[:, 0], bins=50, color='coral', edgecolor='white')
    axes[1].set_xlabel("F_total [N]")
    axes[1].set_ylabel("Cantidad de puntos")
    axes[1].set_title("Distribución de fuerza (output)")
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig("distribucion_dataset.png", dpi=150)
    plt.show()
    
    print(f"Tiempo generación datos: {time.time() - t0:.2f}s")
    
    # ── 2. Preparar dataset ───────────────────────────────────
    t0 = time.time()
    X_train, y_train, X_val, y_val, stats = preparar_dataset(X, y, traj_id)
    print(f"Tiempo preparación dataset: {time.time() - t0:.2f}s")

    # ── 3. Crear la red ───────────────────────────────────────
    t0 = time.time()
    print("\n" + "=" * 65)
    print("ENTRENAMIENTO")
    print("=" * 65)
    modelo = RedBHA()
    
    # ── 4. Entrenar ───────────────────────────────────────────
    
    historial_train, historial_val = entrenar(
        modelo, X_train, y_train, X_val, y_val
    )
    print(f"Tiempo entrenamiento: {time.time() - t0:.2f}s")

        # ── 5. Guardar modelo ─────────────────────────────────────
    torch.save({
        'model_state_dict': modelo.state_dict(),
        'stats': stats,
        'capas': ARQUITECTURA
    }, "modelo_red_k_F_UCS_ROP_RPM.pth")
    print("\nModelo guardado en: modelo_red_k_F_UCS_ROP_RPM.pth")
    
    # ── 6. Ejemplo Sist control ────────────────────────
    t0 = time.time()
    print("\n" + "=" * 65)
    print("EJEMPLO CONTROL COMPLETO")
    print("=" * 65)

    dIds_objetivo = 0.003     # °/mm
    dAds_objetivo = 0.002     # °/mm

    print(f"Curvatura deseada: dI/ds = {dIds_objetivo} °/mm, dA/ds = {dAds_objetivo} °/mm")

    # Agregar posición de ejemplo
    pos_ejemplo = np.array([50.0, 20.0, 300.0])
    rop_ejemplo = 15.0
    rpm_ejemplo = 1500.0
    epsilon_ejemplo = 25.0  # UCS estimado [MPa]

    T1, T2, T3, F_total, kappa = control_completo(
        modelo, stats, dIds_objetivo, dAds_objetivo, pos_ejemplo, rop_ejemplo, rpm_ejemplo, epsilon_ejemplo
    )

    print(f"\nResultados:")
    print(f"  kappa deseado: {kappa:.6f} 1/mm")
    print(f"  F_total:       {F_total:.2f} N")
    print(f"  T1 (  0°):     {T1:.2f} N")
    print(f"  T2 (120°):     {T2:.2f} N")
    print(f"  T3 (240°):     {T3:.2f} N")

    print(f"Tiempo optimizador inverso: {time.time() - t0:.2f}s")


# ── 7. Evaluar ────────────────────────────────────────────
    t0 = time.time()
    print("\n" + "=" * 65)
    print("EVALUACIÓN")
    print("=" * 65)
    evaluar(modelo, X_val, y_val, stats, historial_train, historial_val)
    print(f"Tiempo evaluación: {time.time() - t0:.2f}s")

