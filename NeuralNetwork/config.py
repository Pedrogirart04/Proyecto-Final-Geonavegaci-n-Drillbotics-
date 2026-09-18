"""
config.py — Parámetros del sistema BHA
=======================================
Todas las constantes físicas y de configuración en un solo lugar.
Si cambia un parámetro del sistema, solo se modifica este archivo.
"""

import numpy as np

# ─────────────────────────────────────────────────────────────
# GEOMETRÍA DE LAS PATAS
# ─────────────────────────────────────────────────────────────
# Las 3 patas están separadas 120° entre sí en el plano
# perpendicular al eje de perforación
THETA_PATAS = np.array([0, 2*np.pi/3, 4*np.pi/3])  # [rad] → 0°, 120°, 240°

# ─────────────────────────────────────────────────────────────
# LÍMITES FÍSICOS DE LOS INPUTS
# ─────────────────────────────────────────────────────────────
F_MAX   = 200.0    # Fuerza máxima por pata [N]
# WOB_MIN = 5.0      # Weight on bit mínimo [N]
# WOB_MAX = 50.0     # Weight on bit máximo [N]
# RPM_MIN = 30.0     # RPM mínimas
# RPM_MAX = 150.0    # RPM máximas

# ─────────────────────────────────────────────────────────────
# CURVATURA MÁXIMA ADMISIBLE
# ─────────────────────────────────────────────────────────────
# Derivado de Rmin = 186.43 mm (restricción geométrica del BHA)
R_MIN         = 186.43                          # [mm]
KAPPA_MAX     = 1.0 / R_MIN                    # [1/mm] ≈ 5.36e-3
KAPPA_MAX_DEG = KAPPA_MAX * (180.0 / np.pi)    # [°/mm] ≈ 0.307



# Propiedades de la sarta (tubo de aluminio)
SARTA_D_EXT = 12.7       # Diámetro exterior [mm]
SARTA_D_INT = 9.5        # Diámetro interior [mm]
SARTA_L     = 770.0      # Longitud libre [mm]
SARTA_E     = 69000.0    # Módulo de Young aluminio [MPa = N/mm²]

# Momento de inercia y rigidez
SARTA_I = (np.pi / 64) * (SARTA_D_EXT**4 - SARTA_D_INT**4)
SARTA_K = (3 * SARTA_E * SARTA_I) / (SARTA_L**3)  # k = 3EI/L³ [N/mm]


# ─────────────────────────────────────────────────────────────
# CAMINO AL ARCHIVO DE DATASET
# ─────────────────────────────────────────────────────────────
DATASET_PATH = "dataset_nn.csv"

# ─────────────────────────────────────────────────────────────
# HIPERPARÁMETROS DE LA RED
# ─────────────────────────────────────────────────────────────
ARQUITECTURA = [2, 64, 64, 64, 1]   # [n_inputs, oculta1, oculta2, oculta3, n_outputs]
LR_ADAM      = 1e-3                  # Learning rate para fase Adam
N_EPOCHS     = 500                  # Epochs de Adam
BATCH_SIZE   = 256                   # Tamaño de mini-batch
N_LBFGS      = 100                   # Iteraciones de L-BFGS
FRACCION_VAL = 0.2                   # Fracción del dataset para validación
N_PUNTOS     = 15000                 # Puntos sintéticos a generar
