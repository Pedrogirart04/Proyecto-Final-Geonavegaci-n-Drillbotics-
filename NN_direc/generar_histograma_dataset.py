"""
generar_histograma_dataset.py -- Panel de histogramas del dataset de
entrenamiento de Fase B (ROP, RPM, UCS, kappa).
===========================================================================

Para correr: python generar_histograma_dataset.py
Requiere: dataset_F_k_ROP_RPM_UCS.csv en la misma carpeta (generado por
generar_dataset_v2.m). 
Guarda automáticamente: figures/distribucion_dataset_v2.png
(crea la carpeta "figures" si no existe, al lado de donde corras el script)
"""

import os
import pandas as pd
import matplotlib.pyplot as plt

DATASET_PATH = "dataset_F_k_ROP_RPM_UCS.csv"
OUTPUT_PATH = "figures/distribucion_dataset_v2.png"

df = pd.read_csv(DATASET_PATH)

os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)

variables = [
    ("kappa_1_mm", r"Curvatura $\kappa$ [1/mm]"),
    ("ROP_mm_min", "ROP [mm/min]"),
    ("RPM", "RPM"),
    ("epsilon_MPa", "UCS [MPa]"),
]

fig, axes = plt.subplots(2, 2, figsize=(10, 8))

for ax, (col, label) in zip(axes.flat, variables):
    ax.hist(df[col], bins=40, color="#4472C4", edgecolor="black", linewidth=0.3)
    ax.set_xlabel(label)
    ax.set_ylabel("Frecuencia")
    ax.set_title(label)

fig.suptitle("Distribución del dataset de entrenamiento (Fase B)", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(OUTPUT_PATH, dpi=200)
print(f"Guardado: {OUTPUT_PATH}")