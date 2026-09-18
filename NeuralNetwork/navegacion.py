"""
navegacion.py — Navegación y ciclo de control
===============================================
Método de guiado: Lookahead Point
  - Apunta a un punto de la trayectoria ideal adelante del trepano
  - Corrección de posición y dirección emergen naturalmente
  - Un solo parámetro de tuning: ds_ahead

Contiene:
  - dead_reckoning()             : estimación de posición
  - calcular_curvatura_deseada() : lookahead → curvatura objetivo
  - ciclo_control()              : un ciclo completo de control
"""

import numpy as np
from config import KAPPA_MAX_DEG
from optimizador import control_completo


# ─────────────────────────────────────────────────────────────
# PARÁMETRO DE TUNING
# ─────────────────────────────────────────────────────────────
DS_AHEAD = 30  # Distancia de lookahead [mm]
                  # Muy chico (5mm): correcciones agresivas, posible oscilación
                  # Muy grande (50mm): respuesta lenta
                  # Razonable para este sistema: 15-25mm


# ─────────────────────────────────────────────────────────────
# DEAD RECKONING — Estimación de posición
# ─────────────────────────────────────────────────────────────

def dead_reckoning(pos_actual, I_deg, A_deg, ds):
    """
    Estima la nueva posición integrando la cinemática.

    Usa las ecuaciones:
      dx = sin(I) * cos(A) * ds
      dy = sin(I) * sin(A) * ds
      dz = cos(I) * ds

    Args:
        pos_actual : [x, y, z] actual [mm]
        I_deg      : inclinación actual [°]
        A_deg      : azimut actual [°]
        ds         : avance en profundidad [mm]

    Returns:
        pos_nueva : [x, y, z] estimado [mm]
    """
    I = np.radians(I_deg)
    A = np.radians(A_deg)

    dx = np.sin(I) * np.cos(A) * ds
    dy = np.sin(I) * np.sin(A) * ds
    dz = np.cos(I) * ds

    return pos_actual + np.array([dx, dy, dz])


# ─────────────────────────────────────────────────────────────
# CURVATURA DESEADA — Método Lookahead Point
# ─────────────────────────────────────────────────────────────

def calcular_curvatura_deseada(pos_actual, I_actual, A_actual,
                                trayectoria, z_actual,
                                ds_ahead=DS_AHEAD):
    """
    Calcula la curvatura deseada apuntando a un punto de la
    trayectoria que está ds_ahead mm adelante del trepano.

    Método:
      1. Calcula z_look = z_actual + ds_ahead
      2. Evalúa la trayectoria ideal en z_look → P_target
      3. Calcula el vector desde posición actual hacia P_target
      4. Extrae I_deseado y A_deseado de ese vector
      5. La curvatura deseada es la diferencia angular / ds_ahead

    Args:
        pos_actual  : [x, y, z] actual [mm]
        I_actual    : inclinación actual [°]
        A_actual    : azimut actual [°]
        trayectoria : instancia de TrayectoriaIdeal
        z_actual    : profundidad actual [mm]
        ds_ahead    : distancia de lookahead [mm]

    Returns:
        dIds_deseado : tasa de cambio de inclinación deseada [°/mm]
        dAds_deseado : tasa de cambio de azimut deseada [°/mm]
    """
    # Zona muerta — no corregir si el error es muy chico
    pos_ideal = trayectoria.evaluar(z_actual)
    error_lateral = np.linalg.norm(pos_actual[:2] - pos_ideal[:2])
    if error_lateral < 0.5:
        return 0.0, 0.0
    
    # Paso 1 — Punto de referencia adelante
    z_look = min(z_actual + ds_ahead, trayectoria.z_max)
    P_target = trayectoria.evaluar(z_look)

    # Paso 2 — Vector hacia el target
    delta = P_target - pos_actual
    delta_x = delta[0]
    delta_y = delta[1]
    delta_z = delta[2]

    dist = np.linalg.norm(delta)

    # Si estamos prácticamente en el target, no corregir
    if dist < 1e-6:
        return 0.0, 0.0

    # Paso 3 — Inclinación y azimut deseados
    I_deseado = np.degrees(np.arccos(np.clip(delta_z / dist, -1, 1)))
    A_deseado = np.degrees(np.arctan2(delta_y, delta_x))

    # Paso 4 — Curvatura deseada = diferencia angular / lookahead
    dIds_deseado = (I_deseado - I_actual) / ds_ahead
    dAds_deseado = (A_deseado - A_actual) / ds_ahead

    # Paso 5 —  Limitar curvatura para evitar sobrecompensación
    kappa_max_control = 0.5 / ds_ahead  # °/mm — máximo 0.5° de corrección por paso
    kappa = np.sqrt(dIds_deseado**2 + dAds_deseado**2)
    if kappa > kappa_max_control:
        factor = kappa_max_control / kappa
        dIds_deseado *= factor
        dAds_deseado *= factor

    return dIds_deseado, dAds_deseado


# ─────────────────────────────────────────────────────────────
# CICLO DE CONTROL COMPLETO
# ─────────────────────────────────────────────────────────────

def ciclo_control(modelo, stats, trayectoria,
                  pos_actual, I_actual, A_actual, z_actual, epsilon):
    """
    Ejecuta un ciclo completo de control:
      1. Calcula curvatura deseada (lookahead)
      2. Obtiene fuerzas en pads (red + geometría)
      3. Calcula error lateral para monitoreo

    Args:
        modelo, stats : modelo entrenado y estadísticas de normalización
        trayectoria   : instancia de TrayectoriaIdeal
        pos_actual    : [x, y, z] actual [mm]
        I_actual, A_actual : inclinación y azimut actuales [°]
        z_actual      : profundidad actual [mm]

    Returns:
        F_pads    : [T1, T2, T3] fuerzas en cada pad [N]
        error_pos : distancia lateral al punto ideal [mm]
        curv_des  : (dI/ds, dA/ds) curvatura comandada [°/mm]
    """
    # Paso 1 — Lookahead → curvatura deseada
    dIds_des, dAds_des = calcular_curvatura_deseada(
        pos_actual, I_actual, A_actual,
        trayectoria, z_actual
    )

    # Paso 2 — Red neuronal + descomposición en pads
    T1, T2, T3, F_total, kappa = control_completo(
        modelo, stats, dIds_des, dAds_des, pos_actual, epsilon
    )

    # Paso 3 — Error lateral para monitoreo (no afecta el control)
    pos_ideal = trayectoria.evaluar(z_actual)
    error_pos = np.linalg.norm(pos_actual[:2] - pos_ideal[:2])

    return [T1, T2, T3], error_pos, (dIds_des, dAds_des)