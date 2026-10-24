"""
probar_red.py  Prueba de adaptacion de UCS en 5 pasos (Fase B direccional)
================================================================================

Mecanica de cada paso (igual a la que usa simular_perforacion):
  1. Con el UCS_est actual, se calcula la fuerza necesaria para lograr
     k_comandada (evaluar_red_roca).
  2. Esa misma fuerza se aplica a una roca con el UCS_real de ese paso,
     lo que da el kappa_real resultante (calcular_kappa_real) — si UCS_est
     != UCS_real, kappa_real se desvia de k_comandada.
  3. Con kappa_real y la fuerza aplicada, se re-estima el UCS (estimar_epsilon)
     para usarlo como UCS_est del paso siguiente.

Para correr: python probar_red.py
Requiere: modelo_red_k_F_UCS_ROP_RPM.pth (generado por main.py)
"""

import numpy as np

from control import cargar_modelo, calcular_kappa_real, ROP_SIM, RPM_SIM
from optimizador import evaluar_red_roca
from config import KAPPA_MAX_CONTROL_DEG

# ─────────────────────────────────────────────────────────────
# PARAMETROS DEL TEST
# ─────────────────────────────────────────────────────────────
N_PASOS = 5
DS_PASO = 5.0  # [mm] — mismo paso que usa simular_perforacion

# Curvatura comandada, FIJA en los 5 pasos. Se usa como valor "razonable" el
# límite de corrección por paso del controlador real (KAPPA_MAX_CONTROL_DEG,
# en °/mm), convertido a 1/mm — es el máximo que el controlador real llega a
# comandar cuando el error lateral satura la rampa (ver navegacion.py).
KAPPA_COMANDADA = KAPPA_MAX_CONTROL_DEG * np.pi / 180  # [1/mm] <<<< AJUSTAR

UCS_EST_INICIAL = 25.0   # [MPa] — mismo valor inicial que usa simular_perforacion

# Perfil sinusoidal del UCS real: UCS_real(paso) = 25 + 5*sin(2π·paso/N_pasos + fase)
UCS_CENTRO   = 25.0  # [MPa]
UCS_AMPLITUD = 1.0   # [MPa] -> rango [20,30]
SEMILLA_FASE = None  #


def ucs_real_de_paso(paso, n_pasos, fase,UCS_LAST):
    return UCS_LAST + UCS_AMPLITUD * np.sin(2 * np.pi * paso / n_pasos + fase)


def estimar_epsilon_rango_completo(modelo, stats, kappa_real, rop, rpm, F_roca_aplicada, eps_min, eps_max):
    F_at_min = evaluar_red_roca(modelo, stats, kappa_real, rop, rpm, eps_min)
    F_at_max = evaluar_red_roca(modelo, stats, kappa_real, rop, rpm, eps_max)

    if F_roca_aplicada <= min(F_at_min, F_at_max):
        return eps_min
    if F_roca_aplicada >= max(F_at_min, F_at_max):
        return eps_max

    e_lo, e_hi = eps_min, eps_max
    for _ in range(50):
        e_mid = (e_lo + e_hi) / 2
        F_mid = evaluar_red_roca(modelo, stats, kappa_real, rop, rpm, e_mid)
        if F_mid < F_roca_aplicada:
            e_lo = e_mid
        else:
            e_hi = e_mid

    return (e_lo + e_hi) / 2


if __name__ == "__main__":

    modelo, stats = cargar_modelo("modelo_red_k_F_UCS_ROP_RPM.pth")
    eps_min_red = float(stats['X_min'][3])
    eps_max_red = float(stats['X_max'][3])

    rng = np.random.default_rng(SEMILLA_FASE)
    fase_random = rng.uniform(0, 2 * np.pi)

    print("=" * 65)
    print("PRUEBA DE ADAPTACIÓN DE UCS — 5 PASOS")
    print("=" * 65)
    print(f"ROP={ROP_SIM} mm/min, RPM={RPM_SIM}, k_comandada={KAPPA_COMANDADA:.6f} 1/mm")
    print(f"UCS_est inicial={UCS_EST_INICIAL} MPa, fase UCS_real={fase_random:.3f} rad")
    print(f"Rango UCS de la red: [{eps_min_red:.2f}, {eps_max_red:.2f}] MPa")
    print("=" * 65)

    UCS_est = UCS_EST_INICIAL

    for paso in range(1, N_PASOS + 1):
        s = paso * DS_PASO
        if paso == 1:
            UCS_LAST = UCS_CENTRO
        UCS_real = ucs_real_de_paso(paso, N_PASOS, fase_random,UCS_LAST)
        UCS_LAST = UCS_real
        error_rel_UCS = 100*(UCS_est - UCS_real) / UCS_real

        # Paso 1 — fuerza necesaria para lograr k_comandada, con el UCS
        # estimado actual (es lo único que el controlador real conoce)
        F_comandada = evaluar_red_roca(modelo, stats, KAPPA_COMANDADA, ROP_SIM, RPM_SIM, UCS_est)

        # Paso 2 — esa fuerza, aplicada a la roca con el UCS real, da el
        # kappa que realmente se logra (puede no ser el comandado)
        kappa_real = calcular_kappa_real(modelo, stats, F_comandada, ROP_SIM, RPM_SIM, UCS_real)

        error_rel_kappa = 100*(kappa_real - KAPPA_COMANDADA) / KAPPA_COMANDADA

        print(f"s = {s:.0f} mm")
        print(f"  UCS_est = {UCS_est:.3f} MPa ; UCS_real = {UCS_real:.3f} MPa ; Error_Rel_UCS = {error_rel_UCS:.4f}%")
        print(f"  F_comandada = {F_comandada:.3f} N")
        print(f"  k_comandada = {KAPPA_COMANDADA:.6f} 1/mm ; kappa_real = {kappa_real:.6f} 1/mm ; Error_rel_kappa = {error_rel_kappa:.4f}%")
        print()

        # Paso 3 — con kappa_real y la fuerza aplicada, re-estimar el UCS
        # para usarlo como UCS_est del paso siguiente
        UCS_est = estimar_epsilon_rango_completo(modelo, stats, kappa_real, ROP_SIM, RPM_SIM, F_comandada,
                                                  eps_min_red, eps_max_red)