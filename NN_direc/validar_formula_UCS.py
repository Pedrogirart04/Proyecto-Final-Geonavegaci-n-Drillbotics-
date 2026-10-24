"""
validar_formula_UCS.py — Validación 2 (Fase C): comparación directa
vs. la fórmula analítica, en una grilla sistemática.

Pegar en NN_vert/ y correr: python validar_formula_UCS.py
"""
import numpy as np
from probar_red_UCS import cargar_modelo, predecir_UCS, wob_formula

ROP_VALORES = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0]
RPM_VALORES = [1000.0, 1250.0, 1500.0, 1750.0, 2000.0]
UCS_VALORES = [20.0, 22.5, 25.0, 27.5, 30.0]

if __name__ == "__main__":
    modelo, stats = cargar_modelo()

    print("=" * 75)
    print("VALIDACIÓN 2 (Fase C) — RED vs FÓRMULA ANALÍTICA")
    print("=" * 75)
    print(f"{'ROP':>8} {'RPM':>8} {'UCS_real':>10} | {'WOB':>10} {'UCS_red':>10} {'Desv.':>9} {'Desv.%':>9}")
    print("-" * 75)

    desviaciones, desviaciones_pct = [], []
    for rop in ROP_VALORES:
        for rpm in RPM_VALORES:
            for ucs_real in UCS_VALORES:
                wob = wob_formula(rop, rpm, ucs_real)
                ucs_red = predecir_UCS(modelo, stats, rop, rpm, wob)
                desv = ucs_red - ucs_real
                desv_pct = desv / ucs_real * 100
                desviaciones.append(desv)
                desviaciones_pct.append(desv_pct)
                print(f"{rop:8.2f} {rpm:8.1f} {ucs_real:10.2f} | {wob:10.3f} {ucs_red:10.3f} "
                      f"{desv:+9.3f} {desv_pct:+8.3f}%")

    desviaciones = np.array(desviaciones)
    desviaciones_pct = np.array(desviaciones_pct)
    print("-" * 75)
    print(f"Total de puntos: {len(desviaciones)}")
    print(f"Desviación absoluta — media: {np.mean(np.abs(desviaciones)):.4f} MPa | "
          f"máxima: {np.max(np.abs(desviaciones)):.4f} MPa")
    print(f"Desviación %        — media: {np.mean(np.abs(desviaciones_pct)):.4f}% | "
          f"máxima: {np.max(np.abs(desviaciones_pct)):.4f}%")