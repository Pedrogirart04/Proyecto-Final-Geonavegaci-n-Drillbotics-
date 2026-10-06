"""
config.py — Parámetros del sistema de estimación de UCS (Fase C)
===================================================================
Red inversa: (ROP, RPM, WOB) -> UCS. Ver fase-c-plan.md en el proyecto
para el contexto completo (por qué existe esta red, qué objetivo real
cumple).
"""

# ─────────────────────────────────────────────────────────────
# GEOMETRÍA DEL TRÉPANO
# ─────────────────────────────────────────────────────────────
A_TREPANO = 1.5 * 25.4 / 2   # Radio del trépano [mm] (igual que Fase B)

# ─────────────────────────────────────────────────────────────
# CAMINO AL ARCHIVO DE DATASET
# ─────────────────────────────────────────────────────────────
DATASET_PATH = "dataset_UCS_sintetico.csv"   # <<<< cambiar a CSV real cuando esté

# ─────────────────────────────────────────────────────────────
# HIPERPARÁMETROS DE LA RED
# ─────────────────────────────────────────────────────────────
# Arquitectura deliberadamente CHICA: el dataset sintético tiene 1000
# puntos, pero el real va a tener solo decenas (n probetas x n^2 combi-
# naciones de ROP/RPM). Una red tan grande como la de Fase B (64,64,64)
# memorizaría el dataset real en vez de generalizar. 3 inputs -> problema
# simple, no hace falta mucha capacidad.
ARQUITECTURA = [3, 8, 8, 1]   # [n_inputs, oculta1, oculta2, n_outputs] <<<< AJUSTAR
LR_ADAM      = 1e-3            # Learning rate para fase Adam
N_EPOCHS     = 500             # Epochs de Adam
BATCH_SIZE   = 64              # <<<< AJUSTAR: dataset mucho más chico que Fase B (256)
N_LBFGS      = 100              # Iteraciones de L-BFGS
FRACCION_VAL = 0.2              # Fracción de PROBETAS para validación