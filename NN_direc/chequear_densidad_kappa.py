"""
chequear_densidad_kappa.py — Diagnóstico de densidad de datos de entrenamiento
cerca del extremo bajo de kappa, para entender el error visto en la Validación 2
(Perneder) en kappa=0.0010.

Pegar en NN_direc/, al lado de dataset_F_k_ROP_RPM_UCS.csv, y correr.
"""
import pandas as pd

df = pd.read_csv("datos/dataset_F_k_ROP_RPM_UCS.csv")
kappa = df["kappa_1_mm"]

print(f"Total de puntos en el dataset: {len(kappa)}")
print(f"kappa: min={kappa.min():.5f}  max={kappa.max():.5f}")
print(f"Percentiles: 1%={kappa.quantile(0.01):.5f}  5%={kappa.quantile(0.05):.5f}  "
      f"10%={kappa.quantile(0.10):.5f}  25%={kappa.quantile(0.25):.5f}")

print("\nCantidad de puntos por debajo de distintos umbrales:")
for umbral in [0.0010, 0.0015, 0.0020, 0.0025, 0.0030]:
    n = (kappa < umbral).sum()
    print(f"  kappa < {umbral:.4f}: {n} puntos ({100*n/len(kappa):.3f}%)")

print("\nLos 20 puntos con menor kappa en el dataset real (para ver la forma real de F cerca de 0):")
print(df.nsmallest(20, "kappa_1_mm")[["kappa_1_mm", "F_roca_N", "ROP_mm_min", "RPM", "epsilon_MPa"]]
      .to_string(index=False))

import matplotlib.pyplot as plt
plt.figure(figsize=(8, 5))
plt.hist(kappa, bins=100)
plt.axvline(0.0010, color='r', linestyle='--', label='kappa=0.0010 (punto problemático)')
plt.xlabel("kappa [1/mm]")
plt.ylabel("Cantidad de puntos")
plt.title("Densidad de kappa en el dataset de entrenamiento")
plt.legend()
plt.savefig("histograma_kappa.png")
plt.show()