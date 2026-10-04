"""
optimizador.py — Descomposición en pads con histéresis, filtro angular + control completo
"""

import torch
import numpy as np
from datos import desnormalizar_y
from config import F_MAX, KAPPA_MAX_DEG

# ─────────────────────────────────────────────────────────────
# ESTADO GLOBAL (Sectores y Filtro Paso Bajo)
# ─────────────────────────────────────────────────────────────
_sector_actual = None
MARGEN_HISTERESIS = 5.0  # grados

# Estado del filtro vectorial
_ux_prev = None
_uy_prev = None
ALPHA_FILTRO_THETA = 0.1  # Factor de suavizado: 0.1 (muy suave/lento) a 1.0 (sin filtro)


def resetear_sector():
    """Llamar al inicio de cada simulación para reiniciar el estado."""
    global _sector_actual, _ux_prev, _uy_prev
    _sector_actual = None
    _ux_prev = None
    _uy_prev = None


def aplicar_filtro_vectorial_theta(theta_deg, alpha=ALPHA_FILTRO_THETA):
    """
    Aplica un filtro paso bajo IIR sobre el ángulo descomponiendo en vectores
    para evitar el problema de discontinuidad en 0°/360°.
    """
    global _ux_prev, _uy_prev

    th_rad = np.radians(theta_deg)
    ux_nuevo = np.cos(th_rad)
    uy_nuevo = np.sin(th_rad)

    if _ux_prev is None or _uy_prev is None:
        _ux_prev = ux_nuevo
        _uy_prev = uy_nuevo
        return theta_deg

    ux_filt = alpha * ux_nuevo + (1.0 - alpha) * _ux_prev
    uy_filt = alpha * uy_nuevo + (1.0 - alpha) * _uy_prev

    _ux_prev = ux_filt
    _uy_prev = uy_filt

    theta_filt_deg = np.degrees(np.arctan2(uy_filt, ux_filt)) % 360
    return theta_filt_deg


def evaluar_red_roca(modelo, stats, kappa, rop, rpm, epsilon):
    """Evalúa la red [kappa, ROP, RPM, epsilon] -> F_roca."""
    modelo.eval()
    kappa_min = float(stats['X_min'][0])
    kappa_max = float(stats['X_max'][0])
    rop_min = float(stats['X_min'][1])
    rop_max = float(stats['X_max'][1])
    rpm_min = float(stats['X_min'][2])
    rpm_max = float(stats['X_max'][2])
    eps_min = float(stats['X_min'][3])
    eps_max = float(stats['X_max'][3])

    rop_norm = (rop - rop_min) / (rop_max - rop_min + 1e-8)
    rpm_norm = (rpm - rpm_min) / (rpm_max - rpm_min + 1e-8)
    eps_norm = (epsilon - eps_min) / (eps_max - eps_min + 1e-8)

    if kappa < kappa_min:
        x_min_input = torch.tensor([[0.0, rop_norm, rpm_norm, eps_norm]], dtype=torch.float32)
        with torch.no_grad():
            F_at_min = desnormalizar_y(modelo(x_min_input), stats).item()
        F_roca = F_at_min * (kappa / kappa_min) if kappa_min > 0 else 0.0
    else:
        kappa_norm = (kappa - kappa_min) / (kappa_max - kappa_min + 1e-8)
        x_input = torch.tensor([[kappa_norm, rop_norm, rpm_norm, eps_norm]], dtype=torch.float32)
        with torch.no_grad():
            F_roca = desnormalizar_y(modelo(x_input), stats).item()

    return max(F_roca, 0.0)


def descomponer_en_pads(F_total, theta_push_deg):
    global _sector_actual

    th_deg = theta_push_deg % 360
    th = np.radians(th_deg)
    margen = MARGEN_HISTERESIS

    if th_deg < 120:
        sector_nuevo = 1
    elif th_deg < 240:
        sector_nuevo = 2
    else:
        sector_nuevo = 3

    if _sector_actual is not None:
        sector_nuevo = _sector_actual

        if _sector_actual == 1:
            if th_deg > 120 + margen:
                sector_nuevo = 2
            if th_deg > 240 + margen:
                sector_nuevo = 3

        elif _sector_actual == 2:
            if th_deg < 120 - margen:
                sector_nuevo = 1
            if th_deg > 240 + margen:
                sector_nuevo = 3

        elif _sector_actual == 3:
            if th_deg < 240 - margen:
                sector_nuevo = 2
            if th_deg < 120 - margen and th_deg > margen:
                sector_nuevo = 1

    _sector_actual = sector_nuevo

    if _sector_actual == 1:
        T1 = F_total * (np.cos(th) + (1/np.sqrt(3)) * np.sin(th))
        T2 = F_total * ((2/np.sqrt(3)) * np.sin(th))
        T3 = 0.0
    elif _sector_actual == 2:
        T1 = 0.0
        T2 = F_total * (-np.cos(th) + (1/np.sqrt(3)) * np.sin(th))
        T3 = F_total * (-np.cos(th) - (1/np.sqrt(3)) * np.sin(th))
    else:
        T1 = F_total * (np.cos(th) - (1/np.sqrt(3)) * np.sin(th))
        T2 = 0.0
        T3 = F_total * (-(2/np.sqrt(3)) * np.sin(th))

    return max(T1, 0.0), max(T2, 0.0), max(T3, 0.0)


def control_completo(modelo, stats, dIds_deseado, dAds_deseado, pos_actual, rop, rpm, epsilon):
    from config import SARTA_K

    kappa_deseado = np.sqrt(dIds_deseado**2 + dAds_deseado**2) * np.pi / 180
    delta = np.sqrt(pos_actual[0]**2 + pos_actual[1]**2)
    F_sarta = SARTA_K * delta

    if kappa_deseado < 1e-8:
        if delta > 1e-8:
            theta_deflexion = np.degrees(np.arctan2(pos_actual[1], pos_actual[0]))
            theta_push = aplicar_filtro_vectorial_theta(theta_deflexion, alpha=ALPHA_FILTRO_THETA)
            T1, T2, T3 = descomponer_en_pads(F_sarta, theta_push)
            return T1, T2, T3, F_sarta, 0.0
        return 0.0, 0.0, 0.0, 0.0, 0.0

    theta_mov = np.degrees(np.arctan2(dAds_deseado, dIds_deseado))
    theta_push_bruto = (theta_mov + 180) % 360
    theta_push = aplicar_filtro_vectorial_theta(theta_push_bruto, alpha=ALPHA_FILTRO_THETA)

    F_roca = evaluar_red_roca(modelo, stats, kappa_deseado, rop, rpm, epsilon)
    F_total = F_roca + F_sarta

    T1, T2, T3 = descomponer_en_pads(F_total, theta_push)

    return T1, T2, T3, F_total, kappa_deseado