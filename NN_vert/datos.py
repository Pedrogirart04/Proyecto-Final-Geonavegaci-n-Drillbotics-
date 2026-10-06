"""
datos.py — Generación de datos y normalización (Fase C — UCS)
================================================================
Contiene:
  - cargar_datos()      : carga el dataset (sintético o real) desde CSV
  - preparar_dataset()   : normaliza y divide en train/val (agrupado por probeta)
  - normalizar_X()       : normaliza inputs para inferencia
  - desnormalizar_y()    : convierte outputs normalizados a unidades reales

El split agrupado por probeta_id es EL MISMO código que usa Fase B con
trayectoria_id (ver NeuralNetwork/datos.py) — a propósito, para que
cuando se reemplace dataset_UCS_sintetico.csv por el CSV con datos reales
de laboratorio (mismas columnas, probeta_id repetido n^2 veces por
probeta real) no haga falta tocar una línea de este archivo.
"""

import numpy as np
import torch
from config import DATASET_PATH, FRACCION_VAL


# ─────────────────────────────────────────────────────────────
# CARGAR DATOS
# ─────────────────────────────────────────────────────────────

def cargar_datos(ruta=DATASET_PATH):
    import pandas as pd

    df = pd.read_csv(ruta)

    X = df[['ROP_mm_min', 'RPM', 'WOB_N']].values   # 3 inputs
    y = df[['UCS_MPa']].values                       # 1 output
    probeta_id = df['probeta_id'].values

    print(f"Dataset cargado: {ruta}")
    print(f"  Puntos totales: {len(X)}")
    print(f"  Probetas:       {len(np.unique(probeta_id))}")
    print(f"  ROP rango:      [{X[:,0].min():.2f}, {X[:,0].max():.2f}] mm/min")
    print(f"  RPM rango:      [{X[:,1].min():.1f}, {X[:,1].max():.1f}]")
    print(f"  WOB rango:      [{X[:,2].min():.2f}, {X[:,2].max():.2f}] N")
    print(f"  UCS rango:      [{y[:,0].min():.2f}, {y[:,0].max():.2f}] MPa")

    return X, y, probeta_id


# ─────────────────────────────────────────────────────────────
# NORMALIZACIÓN Y PREPARACIÓN DEL DATASET
# ─────────────────────────────────────────────────────────────

def preparar_dataset(X, y, probeta_id, fraccion_val=FRACCION_VAL):
    """
    Normaliza inputs y outputs a [0,1] y divide en train/validación.

    El split se hace por PROBETA COMPLETA, no por punto individual: todos
    los puntos de una misma probeta van enteros a train o a validación.
    Con el dataset sintético esto es casi sin efecto (cada "probeta" es
    1 sola fila, UCS continuo via LHS). Con el dataset real SÍ importa:
    una probeta tiene n^2 filas (misma UCS, distintos ROP/RPM) y si se
    mezclaran entre train y val, la validación quedaría contaminada
    (puntos de la misma roca ya vistos en train) y no mediría generali-
    zación a UCS nuevos.

    Args:
        X            : array (N, 3) — inputs sin normalizar [ROP, RPM, WOB]
        y            : array (N, 1) — outputs sin normalizar [UCS]
        probeta_id   : array (N,)   — id de probeta de cada punto
        fraccion_val : fracción de PROBETAS para validación

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

    # Split por probeta completa
    ids_unicos = np.unique(probeta_id)
    n_val_probetas = max(1, int(len(ids_unicos) * fraccion_val))

    ids_perm = np.random.permutation(ids_unicos)
    ids_val = set(ids_perm[:n_val_probetas].tolist())

    mask_val = np.array([pid in ids_val for pid in probeta_id])
    mask_train = ~mask_val

    X_train = torch.tensor(X_norm[mask_train], dtype=torch.float32)
    y_train = torch.tensor(y_norm[mask_train], dtype=torch.float32)
    X_val   = torch.tensor(X_norm[mask_val],   dtype=torch.float32)
    y_val   = torch.tensor(y_norm[mask_val],   dtype=torch.float32)

    print(f"\nDataset:")
    print(f"  Train: {mask_train.sum()} puntos ({len(ids_unicos) - n_val_probetas} probetas)")
    print(f"  Val:   {mask_val.sum()} puntos ({n_val_probetas} probetas)")

    return X_train, y_train, X_val, y_val, stats


def desnormalizar_y(y_norm, stats):
    """Convierte outputs normalizados [0,1] de vuelta a unidades reales (UCS en MPa)."""
    y_min = torch.tensor(stats['y_min'], dtype=torch.float32)
    y_max = torch.tensor(stats['y_max'], dtype=torch.float32)
    return y_norm * (y_max - y_min) + y_min


def normalizar_X(X_np, stats):
    """Normaliza un array de inputs [ROP, RPM, WOB] para usar con el modelo entrenado."""
    X_norm = (X_np - stats['X_min']) / (stats['X_max'] - stats['X_min'] + 1e-8)
    return torch.tensor(X_norm, dtype=torch.float32)