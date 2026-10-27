"""
validar_perneder.py — Validación 2: red vs fórmula analítica de Perneder
================================================================================
Carga los puntos (kappa, ROP, RPM, UCS) y el F_roca ANALÍTICO calculado por
MATLAB (validar_perneder_puntos.m, el mismo cálculo de Perneder que generó
el dataset de entrenamiento) y los compara contra lo que predice la red
para esos mismos puntos. La desviación (predicho - analítico) mide qué tan
bien la red aprendió la física del modelo que generó sus propios datos.

Para correr: python validar_perneder.py
Requiere: modelo_red_k_F_UCS_ROP_RPM.pth, validacion_perneder_puntos.csv
"""

import pandas as pd
from control import cargar_modelo
from optimizador import evaluar_red_roca

if __name__ == "__main__":

    modelo, stats = cargar_modelo("modelos/modelo_red_k_F_UCS_ROP_RPM.pth")
    df = pd.read_csv("validacion_perneder_puntos.csv")

    print("=" * 90)
    print("VALIDACIÓN 2 — RED vs FÓRMULA ANALÍTICA DE PERNEDER")
    print("=" * 90)
    print(f"{'kappa':>9} {'ROP':>7} {'RPM':>7} {'UCS':>6} | {'F_analitico':>12} {'F_red':>10} {'Desv.':>9} {'Desv.%':>8}")
    print("-" * 90)

    for _, fila in df.iterrows():
        kappa = fila['kappa_vals']
        rop   = fila['ROP_vals']
        rpm   = fila['RPM_vals']
        ucs   = fila['UCS_vals']
        F_analitico = fila['F_roca_analitico']

        F_red = evaluar_red_roca(modelo, stats, kappa, rop, rpm, ucs)

        desviacion = F_red - F_analitico
        desviacion_pct = (desviacion / F_analitico) * 100 if F_analitico != 0 else float('nan')

        print(f"{kappa:9.4f} {rop:7.2f} {rpm:7.0f} {ucs:6.2f} | {F_analitico:12.3f} {F_red:10.3f} {desviacion:+9.3f} {desviacion_pct:+7.2f}%")