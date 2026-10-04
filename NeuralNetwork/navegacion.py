"""
navegacion.py — Navegación y ciclo de control
===============================================
Método de guiado: Lookahead Point
  - Apunta a un punto de la trayectoria ideal adelante del trepano
  - Corrección de posición y dirección emergen naturalmente
  - Parámetros de tuning: ds_ahead, error_saturacion, k_d

Contiene:
  - dead_reckoning()             : estimación de posición
  - calcular_curvatura_deseada() : lookahead → curvatura objetivo
  - ciclo_control()              : un ciclo completo de control
"""

import numpy as np
from config import KAPPA_MAX_DEG
from optimizador import control_completo
from config import KAPPA_MAX_DEG, KAPPA_MAX_CONTROL_DEG


# ─────────────────────────────────────────────────────────────
# PARÁMETROS DE TUNING
# ─────────────────────────────────────────────────────────────
DS_AHEAD = 30  # Distancia de lookahead [mm]
                  # Muy chico (5mm): correcciones agresivas, posible oscilación
                  # Muy grande (50mm): respuesta lenta
                  # Razonable para este sistema: 15-25mm

ERROR_SATURACION = 0.8  # [mm] <<<< AJUSTAR: error lateral a partir del cual
                         # se satura en KAPPA_MAX_CONTROL_DEG. Por debajo de
                         # esto, la corrección es proporcional (rampa lineal
                         # desde la zona muerta de 0.5mm). Valor elegido tras
                         # el barrido (mejor resultado en 0.75-0.8mm).

K_D_AMORTIGUACION = 2.0  # [1/mm] <<<< AJUSTAR: fuerza del término derivativo.
                          # Si el error lateral cae rápido de un paso al
                          # siguiente (convergencia), afloja la corrección
                          # para anticipar el sobregiro en vez de esperar a
                          # pasarse de largo. 0 = sin amortiguación (solo P).
                          # Sin tunear todavía — punto de partida.


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
                                ds_ahead=DS_AHEAD, error_saturacion=None,
                                error_anterior=None, k_d=None):
    """
    Calcula la curvatura deseada apuntando a un punto de la
    trayectoria que está ds_ahead mm adelante del trepano.

    Método:
      1. Calcula z_look = z_actual + ds_ahead
      2. Evalúa la trayectoria ideal en z_look → P_target
      3. Calcula el vector desde posición actual hacia P_target
         → eso da la DIRECCIÓN de corrección (I_deseado, A_deseado)
      4. La MAGNITUD de la corrección es proporcional al error lateral,
         con rampa entre la zona muerta y error_saturacion, saturada en
         KAPPA_MAX_CONTROL_DEG (término P).
      5. Si se pasa error_anterior, se resta un término derivativo: si el
         error ya viene cayendo rápido, afloja la corrección para anticipar
         el sobregiro (término D).

    Args:
        pos_actual      : [x, y, z] actual [mm]
        I_actual        : inclinación actual [°]
        A_actual        : azimut actual [°]
        trayectoria     : instancia de TrayectoriaIdeal
        z_actual        : profundidad actual [mm]
        ds_ahead        : distancia de lookahead [mm]
        error_saturacion: [mm] umbral de saturación de la rampa (None = usa el default del módulo)
        error_anterior  : [mm] error lateral del paso de control previo (None = sin término D)
        k_d             : [1/mm] ganancia del término derivativo (None = usa el default del módulo)

    Returns:
        dIds_deseado : tasa de cambio de inclinación deseada [°/mm]
        dAds_deseado : tasa de cambio de azimut deseada [°/mm]
    """
    if error_saturacion is None:
        error_saturacion = ERROR_SATURACION
    if k_d is None:
        k_d = K_D_AMORTIGUACION

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

    # Paso 3 — Inclinación y azimut deseados (dan la DIRECCIÓN)
    I_deseado = np.degrees(np.arccos(np.clip(delta_z / dist, -1, 1)))
    A_deseado = np.degrees(np.arctan2(delta_y, delta_x))

    dI_dir = I_deseado - I_actual
    dA_dir = A_deseado - A_actual
    norm_dir = np.sqrt(dI_dir**2 + dA_dir**2)
    if norm_dir < 1e-8:
        return 0.0, 0.0
    dI_dir /= norm_dir
    dA_dir /= norm_dir

    # Paso 4 — Término proporcional: rampa según error lateral actual
    factor_rampa = np.clip(
        (error_lateral - 0.5) / (error_saturacion - 0.5), 0.0, 1.0
    )
    kappa_objetivo = factor_rampa * KAPPA_MAX_CONTROL_DEG

    # Paso 5 — Término derivativo: si el error ya viene cayendo rápido
    # (convergencia), anticipar el sobregiro y aflojar la corrección antes
    # de pasarse de largo. Si crece o está estable, no se toca.
    if error_anterior is not None:
        delta_error = error_lateral - error_anterior
        amortiguacion = np.clip(-k_d * delta_error, 0.0, 0.9)
        kappa_objetivo *= (1.0 - amortiguacion)

    dIds_deseado = dI_dir * kappa_objetivo
    dAds_deseado = dA_dir * kappa_objetivo

    return dIds_deseado, dAds_deseado


# ─────────────────────────────────────────────────────────────
# CICLO DE CONTROL COMPLETO
# ─────────────────────────────────────────────────────────────

def ciclo_control(modelo, stats, trayectoria,
                  pos_actual, I_actual, A_actual, z_actual, rop, rpm, epsilon,
                  error_saturacion=None, error_anterior=None, k_d=None):
    """
    Ejecuta un ciclo completo de control:
      1. Calcula curvatura deseada (lookahead, con rampa P + amortiguación D)
      2. Obtiene fuerzas en pads (red + geometría)
      3. Calcula error lateral para monitoreo

    Args:
        modelo, stats : modelo entrenado y estadísticas de normalización
        trayectoria   : instancia de TrayectoriaIdeal
        pos_actual    : [x, y, z] actual [mm]
        I_actual, A_actual : inclinación y azimut actuales [°]
        z_actual      : profundidad actual [mm]
        rop, rpm      : ROP y RPM fijos de la corrida (conocidos, no estimados)
        error_saturacion : ver calcular_curvatura_deseada
        error_anterior   : ver calcular_curvatura_deseada
        k_d              : ver calcular_curvatura_deseada

    Returns:
        F_pads    : [T1, T2, T3] fuerzas en cada pad [N]
        error_pos : distancia lateral al punto ideal [mm]
        curv_des  : (dI/ds, dA/ds) curvatura comandada [°/mm]
    """
    dIds_des, dAds_des = calcular_curvatura_deseada(
        pos_actual, I_actual, A_actual,
        trayectoria, z_actual,
        error_saturacion=error_saturacion,
        error_anterior=error_anterior,
        k_d=k_d
    )

    T1, T2, T3, F_total, kappa = control_completo(
        modelo, stats, dIds_des, dAds_des, pos_actual, rop, rpm, epsilon
    )

    pos_ideal = trayectoria.evaluar(z_actual)
    error_pos = np.linalg.norm(pos_actual[:2] - pos_ideal[:2])

    return [T1, T2, T3], error_pos, (dIds_des, dAds_des)