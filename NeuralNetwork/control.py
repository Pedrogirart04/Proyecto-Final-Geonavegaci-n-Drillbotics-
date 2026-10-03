"""
control.py — Loop principal de control
========================================
Incluye:
  - Perfil de ε(z) variable
  - Modelo de planta basado en la red neuronal
  - Ruido de medición del IMU
  - Estimación online de ε por bisección
  - Histéresis en descomposición de pads
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
import sys
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline

from modelo import RedBHA
from navegacion import dead_reckoning, ciclo_control
from datos import desnormalizar_y
from config import SARTA_K, KAPPA_MAX_CONTROL_DEG
from optimizador import resetear_sector, evaluar_red_roca
from trayectoria import TrayectoriaIdeal, generar_waypoints_random

# ─────────────────────────────────────────────────────────────
# PARÁMETROS DE SIMULACIÓN
# ─────────────────────────────────────────────────────────────

# Ruido del IMU — poner 0 para desactivar
RUIDO_IMU_I  = 0
RUIDO_IMU_A  = 0

# Variabilidad del UCS
RUIDO_UCS_LOCAL = 0
N_NODOS_UCS    = 6

# Rango de ε
EPSILON_MIN = 22.0
EPSILON_MAX = 28.0

# Estimación online de ε
ADAPTACION_ACTIVA  = True
VENTANA_SUAVIZADO  = 1    # Promediar últimos N valores de ε estimado


# ─────────────────────────────────────────────────────────────
# PARA SALIDA (TXT)
# ─────────────────────────────────────────────────────────────
class Tee:
    """Escribe la salida tanto en consola como en un archivo de texto."""
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for s in self.streams:
            s.write(data)
    def flush(self):
        for s in self.streams:
            s.flush()





# ─────────────────────────────────────────────────────────────
# CARGA DEL MODELO
# ─────────────────────────────────────────────────────────────

def cargar_modelo(ruta="modelo_bha.pth"):
    checkpoint = torch.load(ruta, weights_only=False)
    modelo = RedBHA(capas=checkpoint['capas'])
    modelo.load_state_dict(checkpoint['model_state_dict'])
    modelo.eval()
    stats = checkpoint['stats']
    print(f"Modelo cargado desde: {ruta}")
    return modelo, stats


# ─────────────────────────────────────────────────────────────
# PERFIL DE EPSILON VARIABLE
# ─────────────────────────────────────────────────────────────

def generar_perfil_epsilon(z_max, semilla=None):
    if semilla is not None:
        rng = np.random.default_rng(semilla)
    else:
        rng = np.random.default_rng()

    z_nodos = np.linspace(0, z_max, N_NODOS_UCS)
    eps_nodos = rng.uniform(EPSILON_MIN, EPSILON_MAX, N_NODOS_UCS)
    spline_eps = CubicSpline(z_nodos, eps_nodos)

    def epsilon_de_z(z):
        eps_suave = float(spline_eps(np.clip(z, 0, z_max)))
        eps_ruido = rng.normal(0, RUIDO_UCS_LOCAL) if RUIDO_UCS_LOCAL > 0 else 0.0
        eps_total = eps_suave + eps_ruido
        return float(np.clip(eps_total, EPSILON_MIN, EPSILON_MAX))

    return epsilon_de_z


# ─────────────────────────────────────────────────────────────
# PLANTA: INVERSIÓN DE LA RED NEURONAL
# ─────────────────────────────────────────────────────────────

def calcular_kappa_real(modelo, stats, F_comandada, epsilon_real):
    """Dada F y ε_real, encuentra κ_real por bisección usando evaluar_red_roca."""
    kappa_min = float(stats['X_min'][0])
    kappa_max = float(stats['X_max'][0])

    F_min = evaluar_red_roca(modelo, stats, kappa_min, epsilon_real)
    if F_comandada <= F_min:
        return kappa_min * (F_comandada / F_min) if F_min > 0 else 0.0

    F_max = evaluar_red_roca(modelo, stats, kappa_max, epsilon_real)
    if F_comandada >= F_max:
        return kappa_max

    k_lo, k_hi = kappa_min, kappa_max
    for _ in range(50):
        k_mid = (k_lo + k_hi) / 2
        if evaluar_red_roca(modelo, stats, k_mid, epsilon_real) < F_comandada:
            k_lo = k_mid
        else:
            k_hi = k_mid

    return (k_lo + k_hi) / 2


# ─────────────────────────────────────────────────────────────
# ESTIMACIÓN DE ε POR BISECCIÓN
# ─────────────────────────────────────────────────────────────

def estimar_epsilon(modelo, stats, kappa_real, F_roca_aplicada):
    """Dado κ_real y F_roca aplicada, encuentra ε por bisección usando evaluar_red_roca."""
    F_at_min = evaluar_red_roca(modelo, stats, kappa_real, EPSILON_MIN)
    F_at_max = evaluar_red_roca(modelo, stats, kappa_real, EPSILON_MAX)

    if F_roca_aplicada <= min(F_at_min, F_at_max):
        return EPSILON_MIN
    if F_roca_aplicada >= max(F_at_min, F_at_max):
        return EPSILON_MAX

    e_lo, e_hi = EPSILON_MIN, EPSILON_MAX
    for _ in range(50):
        e_mid = (e_lo + e_hi) / 2
        F_mid = evaluar_red_roca(modelo, stats, kappa_real, e_mid)
        if F_mid < F_roca_aplicada:
            e_lo = e_mid
        else:
            e_hi = e_mid

    return (e_lo + e_hi) / 2


# ─────────────────────────────────────────────────────────────
# SIMULACIÓN DE PERFORACIÓN
# ─────────────────────────────────────────────────────────────

def simular_perforacion(modelo, stats, semilla_trayectoria=None):
    rng_tray = np.random.default_rng(semilla_trayectoria)
    waypoints_curva = generar_waypoints_random(rng=rng_tray)
    trayectoria = TrayectoriaIdeal(waypoints_curva=waypoints_curva)
    print(f"Waypoints: P1=(0,0,{trayectoria.L_bit}) [fijo], "
          f"P2={waypoints_curva[0]}, P3={waypoints_curva[1]}")
    
    ds_paso    = 5.0
    s_total    = trayectoria.z_max

    epsilon_perfil = generar_perfil_epsilon(s_total, semilla=42)
    resetear_sector()

    # Estado real
    pos_real = np.array([0.0, 0.0, 0.0])
    I_real   = 0.0
    A_real   = 0.0
    s        = 0.0

    # Estado medido
    I_medido = 0.0
    A_medido = 0.0

    # Estimación de ε
    epsilon_estimado = 25.0
    historial_eps_estimados = []  # ventana para suavizar

    # Historial
    historial_pos      = [pos_real.copy()]
    historial_ideal    = [trayectoria.evaluar(0.0)]
    historial_error    = [0.0]
    historial_s        = [0.0]
    historial_F        = []
    historial_eps_real = [epsilon_perfil(0.0)]
    historial_eps_est  = [epsilon_estimado]

    print("\n" + "═" * 85)
    print("SIMULACIÓN DE PERFORACIÓN")
    print("═" * 85)
    print(f"  Ruido IMU: σ_I={RUIDO_IMU_I}°, σ_A={RUIDO_IMU_A}°")
    print(f"  Ruido UCS local: σ={RUIDO_UCS_LOCAL} MPa")
    print(f"  Adaptación ε: {'BISECCIÓN' if ADAPTACION_ACTIVA else 'DESACTIVADA'}")
    print("─" * 85)
    print(f"{'s':>6} | {'Error':>8} | {'I':>7} | {'A':>7} | "
          f"{'T1':>7} | {'T2':>7} | {'T3':>7} | {'ε_real':>6} | {'ε_est':>6}")
    print("─" * 85)
    
    historial_I = []
    historial_A = []
    VENTANA_IMU = 5
    
    while s < s_total:
        
        s += ds_paso
        eps_real = epsilon_perfil(s)

        if s <= trayectoria.L_bit or pos_real[2] > trayectoria.z_max - ds_paso:
            T1, T2, T3 = 0.0, 0.0, 0.0
            error_pos = 0.0
        else:
            # ── Control ──────────────────────────────────────
            F_opt, error_pos, curv_des = ciclo_control(
                modelo, stats, trayectoria,
                pos_real, I_medido, A_medido, pos_real[2],
                epsilon_estimado
            )
            T1, T2, T3 = F_opt

            # ── Planta ───────────────────────────────────────
            kappa_comandado = np.sqrt(curv_des[0]**2 + curv_des[1]**2) * np.pi / 180

            if kappa_comandado > 1e-8:
                # Se usa evaluar_red_roca en vez de llamar a PyTorch a mano
                F_roca_comandada = evaluar_red_roca(modelo, stats, kappa_comandado, epsilon_estimado)

                kappa_real = calcular_kappa_real(modelo, stats, F_roca_comandada, eps_real)

                factor = (kappa_real / kappa_comandado) * (180 / np.pi)
                dIds_real = curv_des[0] * factor
                dAds_real = curv_des[1] * factor

                
                # ── Estimación de ε por bisección ─────────────
                if ADAPTACION_ACTIVA and kappa_real > 1e-8:
                    eps_instantaneo = estimar_epsilon(
                        modelo, stats, kappa_real, F_roca_comandada
                    )

                    historial_eps_estimados.append(eps_instantaneo)
                    if len(historial_eps_estimados) > VENTANA_SUAVIZADO:
                        historial_eps_estimados.pop(0)

                    epsilon_estimado = np.mean(historial_eps_estimados)
                    epsilon_estimado = np.clip(epsilon_estimado, EPSILON_MIN, EPSILON_MAX)
            else:
                dIds_real = 0.0
                dAds_real = 0.0
                F_roca_comandada = 0.0
                kappa_real = 0.0

            print(f"  DEBUG s={s:.0f} | κ_cmd={kappa_comandado:.6f} | κ_real={kappa_real:.6f} | "
                                f"F_cmd={F_roca_comandada:.2f} | ε_est={epsilon_estimado:.2f} | ε_real={eps_real:.2f} | "
                                f"θ_push={(np.degrees(np.arctan2(curv_des[1], curv_des[0])) + 180) % 360:.1f}°")
        
            I_real += dIds_real * ds_paso
            A_real += dAds_real * ds_paso

            I_medido = I_real + np.random.normal(0, RUIDO_IMU_I) if RUIDO_IMU_I > 0 else I_real
            A_medido = A_real + np.random.normal(0, RUIDO_IMU_A) if RUIDO_IMU_A > 0 else A_real
            
            historial_I.append(I_medido)
            historial_A.append(A_medido)
            if len(historial_I) > VENTANA_IMU:
                historial_I.pop(0)
                historial_A.pop(0)

            I_medido = sum(historial_I) / len(historial_I)
            A_medido = sum(historial_A) / len(historial_A)

        pos_real = dead_reckoning(pos_real, I_real, A_real, ds_paso)

        historial_pos.append(pos_real.copy())
        historial_ideal.append(trayectoria.evaluar(pos_real[2]))
        historial_error.append(error_pos)
        historial_s.append(s)
        historial_F.append([T1, T2, T3])
        historial_eps_real.append(eps_real)
        historial_eps_est.append(epsilon_estimado)

        if int(s) % 50 == 0 or s >= s_total:
            print(f"{s:6.1f} | {error_pos:8.3f} | {I_real:7.2f} | {A_real:7.2f} | "
                  f"{T1:7.1f} | {T2:7.1f} | {T3:7.1f} | {eps_real:6.1f} | {epsilon_estimado:6.1f}")

    graficar_simulacion(historial_pos, historial_ideal,
                        historial_error, historial_s, historial_F,
                        historial_eps_real, historial_eps_est,
                        trayectoria.waypoints_curva)


# ─────────────────────────────────────────────────────────────
# GRÁFICOS
# ─────────────────────────────────────────────────────────────

def graficar_simulacion(hist_pos, hist_ideal, hist_error, hist_s, hist_F,
                        hist_eps_real, hist_eps_est, waypoints):
    pos   = np.array(hist_pos)
    ideal = np.array(hist_ideal)
    error = np.array(hist_error)
    s_arr = np.array(hist_s)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Simulación de Perforación con Control NN", fontsize=14)

    axes[0,0].plot(ideal[:, 0], ideal[:, 2], 'b--', lw=2, label="Ideal")
    axes[0,0].plot(pos[:, 0],   pos[:, 2],   'r-',  lw=1.5, label="Real")
    axes[0,0].plot(waypoints[:, 0], waypoints[:, 2], 'ko', markersize=8, label="Waypoints")
    axes[0,0].set_xlabel("x [mm]"); axes[0,0].set_ylabel("z [mm]")
    axes[0,0].set_title("Trayectoria XZ"); axes[0,0].invert_yaxis()
    axes[0,0].legend(); axes[0,0].grid(True)

    axes[0,1].plot(ideal[:, 1], ideal[:, 2], 'b--', lw=2, label="Ideal")
    axes[0,1].plot(pos[:, 1],   pos[:, 2],   'r-',  lw=1.5, label="Real")
    axes[0,1].plot(waypoints[:, 1], waypoints[:, 2], 'ko', markersize=8, label="Waypoints")
    axes[0,1].set_xlabel("y [mm]"); axes[0,1].set_ylabel("z [mm]")
    axes[0,1].set_title("Trayectoria YZ"); axes[0,1].invert_yaxis()
    axes[0,1].legend(); axes[0,1].grid(True)

    axes[0,2].plot(s_arr, hist_eps_real, 'g-', lw=1.5, label="ε real")
    axes[0,2].plot(s_arr, hist_eps_est,  'r--', lw=1.5, label="ε estimado")
    axes[0,2].set_xlabel("Profundidad s [mm]"); axes[0,2].set_ylabel("ε [MPa]")
    axes[0,2].set_title("UCS real vs estimado")
    axes[0,2].legend(); axes[0,2].grid(True)

    axes[1,0].plot(s_arr, error, 'k-', lw=1.5)
    axes[1,0].set_xlabel("Profundidad s [mm]"); axes[1,0].set_ylabel("Error lateral [mm]")
    axes[1,0].set_title("Error lateral vs profundidad"); axes[1,0].grid(True)

    if len(hist_F) > 0:
        F_arr = np.array(hist_F)
        s_F   = s_arr[1:len(F_arr)+1]
        axes[1,1].plot(s_F, F_arr[:, 0], label="T1 (0°)",   lw=1.5)
        axes[1,1].plot(s_F, F_arr[:, 1], label="T2 (120°)", lw=1.5)
        axes[1,1].plot(s_F, F_arr[:, 2], label="T3 (240°)", lw=1.5)
        axes[1,1].set_xlabel("Profundidad s [mm]"); axes[1,1].set_ylabel("Fuerza [N]")
        axes[1,1].set_title("Fuerzas aplicadas por pata")
        axes[1,1].legend(); axes[1,1].grid(True)

    axes[1,2].axis('off')

    plt.tight_layout()
    plt.savefig("simulacion_perforacion.png", dpi=150)
    plt.show()

    '''
    fig_3d = plt.figure(figsize=(10, 8))
    ax = fig_3d.add_subplot(111, projection='3d')
    ax.plot(ideal[:, 0], ideal[:, 1], ideal[:, 2], 'b--', lw=2, label="Ideal")
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'r-', lw=1.5, label="Real")
    ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
            'ko', markersize=8, label="Waypoints")
    ax.set_xlabel("X [mm]"); ax.set_ylabel("Y [mm]"); ax.set_zlabel("Z [mm]")
    ax.set_title("Trayectoria 3D: Ideal vs Real"); ax.invert_zaxis()
    ax.legend()
    plt.savefig("simulacion_3d.png", dpi=150)
    plt.show()
    '''


if __name__ == "__main__":
    nombre_log = f"salida_kmax{KAPPA_MAX_CONTROL_DEG}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    archivo_log = open(nombre_log, "w", encoding="utf-8", buffering=1)
    sys.stdout = Tee(sys.__stdout__, archivo_log)

    np.random.seed(42)
    modelo, stats = cargar_modelo("modelo_bha.pth")
    simular_perforacion(modelo, stats)

    archivo_log.close()