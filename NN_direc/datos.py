"""
datos.py — Generación de datos y normalización
================================================
Contiene:
  - cargar_datos()     : carga el dataset real desde CSV
  - preparar_dataset()  : normaliza y divide en train/val (agrupado por trayectoria)
  - normalizar_X()      : normaliza inputs para inferencia
  - desnormalizar_y()   : convierte outputs normalizados a unidades reales
"""

import numpy as np
import torch
from config import (DATASET_PATH, FRACCION_VAL)


# ─────────────────────────────────────────────────────────────
# CARGAR DATOS
# ─────────────────────────────────────────────────────────────

def cargar_datos(ruta=DATASET_PATH):
    import pandas as pd

    df = pd.read_csv(ruta)

    X = df[['kappa_1_mm', 'ROP_mm_min', 'RPM', 'epsilon_MPa']].values  # 4 inputs
    y = df[['F_roca_N']].values                                        # 1 output
    traj_id = df['trayectoria_id'].values

    print(f"Dataset cargado: {ruta}")
    print(f"  Puntos totales: {len(X)}")
    print(f"  Trayectorias:   {len(np.unique(traj_id))}")
    print(f"  kappa rango:    [{X[:,0].min():.6f}, {X[:,0].max():.6f}] 1/mm")
    print(f"  ROP rango:      [{X[:,1].min():.2f}, {X[:,1].max():.2f}] mm/min")
    print(f"  RPM rango:      [{X[:,2].min():.1f}, {X[:,2].max():.1f}]")
    print(f"  epsilon rango:  [{X[:,3].min():.2f}, {X[:,3].max():.2f}] MPa")
    print(f"  F_roca rango:   [{y[:,0].min():.2f}, {y[:,0].max():.2f}] N")

    return X, y, traj_id


# ─────────────────────────────────────────────────────────────
# NORMALIZACIÓN Y PREPARACIÓN DEL DATASET
# ─────────────────────────────────────────────────────────────

def preparar_dataset(X, y, traj_id, fraccion_val=FRACCION_VAL):
    """
    Normaliza inputs y outputs a [0,1] y divide en train/validación.

    El split se hace por TRAYECTORIA COMPLETA, no por punto individual:
    todos los puntos de una misma trayectoria van enteros a train o a
    validación. Si se mezclaran puntos de la misma trayectoria entre
    ambos conjuntos, la validación quedaría contaminada (puntos casi
    idénticos ya vistos en train) y no mediría generalización real.

    Args:
        X       : array (N, 4) — inputs sin normalizar [kappa, ROP, RPM, epsilon]
        y       : array (N, 1) — outputs sin normalizar
        traj_id : array (N,)   — id de trayectoria de cada punto
        fraccion_val : fracción de TRAYECTORIAS para validación

    Returns:
        X_train, y_train, X_val, y_val, stats
    """
    X_min = X.min(axis=0)
    X_max = X.max(axis=0)
    y_min = y.min(axis=0)
    y_max = y.max(axis=0)

    stats = {
        'X_min': X_min, 'X_max': X_max,
        'y_min': y_min, 'y_max': y_max
    }

    X_norm = (X - X_min) / (X_max - X_min + 1e-8)
    y_norm = (y - y_min) / (y_max - y_min + 1e-8)

    # Split por trayectoria completa
    ids_unicos = np.unique(traj_id)
    n_val_traj = max(1, int(len(ids_unicos) * fraccion_val))

    ids_perm = np.random.permutation(ids_unicos)
    ids_val = set(ids_perm[:n_val_traj].tolist())

    mask_val = np.array([tid in ids_val for tid in traj_id])
    mask_train = ~mask_val

    X_train = torch.tensor(X_norm[mask_train], dtype=torch.float32)
    y_train = torch.tensor(y_norm[mask_train], dtype=torch.float32)
    X_val   = torch.tensor(X_norm[mask_val],   dtype=torch.float32)
    y_val   = torch.tensor(y_norm[mask_val],   dtype=torch.float32)

    print(f"\nDataset:")
    print(f"  Train: {mask_train.sum()} puntos ({len(ids_unicos) - n_val_traj} trayectorias)")
    print(f"  Val:   {mask_val.sum()} puntos ({n_val_traj} trayectorias)")

    return X_train, y_train, X_val, y_val, stats


def desnormalizar_y(y_norm, stats):
    """Convierte outputs normalizados [0,1] de vuelta a unidades reales."""
    y_min = torch.tensor(stats['y_min'], dtype=torch.float32)
    y_max = torch.tensor(stats['y_max'], dtype=torch.float32)
    return y_norm * (y_max - y_min) + y_min


def normalizar_X(X_np, stats):
    """Normaliza un array de inputs para usar con el modelo entrenado."""
    X_norm = (X_np - stats['X_min']) / (stats['X_max'] - stats['X_min'] + 1e-8)
    return torch.tensor(X_norm, dtype=torch.float32)