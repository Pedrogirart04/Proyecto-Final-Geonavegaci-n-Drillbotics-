"""
validar_cons_fisica.py — Validación 1: consistencia física por input
================================================================================
Barre cada input de la red (kappa, ROP, RPM, epsilon/UCS) por separado,
dejando los otros tres fijos en un valor típico (el punto medio de su rango
de entrenamiento), y mira cómo responde F_roca. Para kappa y UCS el chequeo
es automático (deberían ser monótonamente crecientes: a mayor curvatura
pedida o mayor dureza de roca, más fuerza). Para ROP y RPM no hay una
dirección obvia a priori desde la fórmula de Perneder (lateral, no el
término axial simple de Fase C), así que ahí no se afirma nada, solo se
imprime/grafica para mirarlo a ojo.

Para correr: python validar_consistencia_fisica.py
Requiere: modelo_red_k_F_UCS_ROP_RPM.pth
"""

import numpy as np
import matplotlib.pyplot as plt

from control import cargar_modelo
from optimizador import evaluar_red_roca

N_PUNTOS = 20  # <<<< AJUSTAR

NOMBRES = ['kappa [1/mm]', 'ROP [mm/min]', 'RPM', 'UCS/epsilon [MPa]']


def punto_medio(stats):
    return (stats['X_min'] + stats['X_max']) / 2


def barrer_input(modelo, stats, idx_input):
    """Barre el input idx_input en su rango de entrenamiento, dejando los
    otros 3 fijos en su punto medio. Devuelve (valores, F_roca)."""
    base = punto_medio(stats)
    valores = np.linspace(stats['X_min'][idx_input], stats['X_max'][idx_input], N_PUNTOS)
    F_vals = []
    for v in valores:
        punto = base.copy()
        punto[idx_input] = v
        F = evaluar_red_roca(modelo, stats, punto[0], punto[1], punto[2], punto[3])
        F_vals.append(F)
    return valores, np.array(F_vals)


def es_monotona_creciente(F_vals, tolerancia=1e-6):
    diffs = np.diff(F_vals)
    return np.all(diffs >= -tolerancia)


if __name__ == "__main__":

    modelo, stats = cargar_modelo("modelo_red_k_F_UCS_ROP_RPM.pth")

    fig, axes = plt.subplots(1, 4, figsize=(20, 4))
    fig.suptitle("Validación 1 — Consistencia física por input", fontsize=14)

    print("=" * 70)
    print("VALIDACIÓN 1 — CONSISTENCIA FÍSICA POR INPUT")
    print("=" * 70)

    for idx in range(4):
        valores, F_vals = barrer_input(modelo, stats, idx)

        axes[idx].plot(valores, F_vals, 'o-')
        axes[idx].set_xlabel(NOMBRES[idx])
        axes[idx].set_ylabel("F_roca [N]")
        axes[idx].grid(True)

        print(f"\n{NOMBRES[idx]}:")
        for v, F in zip(valores, F_vals):
            print(f"   {v:10.4f} -> F_roca = {F:8.3f} N")

        if idx in (0, 3):  # kappa y UCS: se espera monótono creciente
            ok = es_monotona_creciente(F_vals)
            estado = "OK (monótono creciente)" if ok else "¡ATENCIÓN! no es monótono creciente"
            axes[idx].set_title(estado)
            print(f"   -> {estado}")
        else:
            axes[idx].set_title("(sin dirección esperada a priori, mirar a ojo)")

    plt.tight_layout()
    plt.savefig("validacion_consistencia_fisica.png", dpi=150)
    plt.show()