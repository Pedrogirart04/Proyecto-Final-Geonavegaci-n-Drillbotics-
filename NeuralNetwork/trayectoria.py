"""
trayectoria.py — Trayectoria ideal con spline cúbico sujeto
=============================================================
Genera la trayectoria interpolando waypoints con spline cúbico
parametrizado por longitud de cuerda, con condición de arranque
vertical (clamped). Consistente con el método del informe y el
código MATLAB (generar_trayectoria.m).

Garantiza continuidad C0, C1 y C2 y tramo rígido vertical
durante los primeros L_bit mm.
"""

import numpy as np
from scipy.interpolate import CubicSpline
from config import KAPPA_MAX_DEG


class TrayectoriaIdeal:
    """
    Trayectoria ideal definida por spline cúbico sujeto que
    interpola los waypoints con condición de arranque vertical.
    """

def __init__(self, waypoints_curva=None, L_bit=120.0):
    """
    Args:
        waypoints_curva : array (2,3) con [x, y, z] de los dos waypoints
                          que definen la parte direccional (P2, P3).
                          El primer waypoint (P1) queda fijo en
                          (0, 0, L_bit): fin del tramo recto y arranque
                          de la curva, con tangente vertical exacta ahí.
        L_bit           : largo del tramo rígido vertical [mm]
    """
    if waypoints_curva is None:
        waypoints_curva = np.array([
            [64.3,   0.0,    360.0],   # P2
            [107.2,  0.0,    520.0],   # P3
        ])

    self.L_bit = L_bit
    self.P0 = np.array([0.0, 0.0, 0.0])
    self.P1 = np.array([0.0, 0.0, L_bit])

    self.waypoints_curva = np.vstack([self.P1, waypoints_curva])  # [P1, P2, P3]
    self.L_recto = self.L_bit

    puntos_curva = self.waypoints_curva
    dists = np.sqrt(np.sum(np.diff(puntos_curva, axis=0)**2, axis=1))
    t_waypoints = np.concatenate([[0], np.cumsum(dists)])

    v_start = (self.P1 - self.P0) / self.L_recto

    v_end = (puntos_curva[-1] - puntos_curva[-2])
    v_end = v_end / (np.linalg.norm(v_end) + 1e-10)

    self.spline_x = CubicSpline(
        t_waypoints, puntos_curva[:, 0],
        bc_type=((1, v_start[0]), (1, v_end[0]))
    )
    self.spline_y = CubicSpline(
        t_waypoints, puntos_curva[:, 1],
        bc_type=((1, v_start[1]), (1, v_end[1]))
    )
    self.spline_z = CubicSpline(
        t_waypoints, puntos_curva[:, 2],
        bc_type=((1, v_start[2]), (1, v_end[2]))
    )

    self.t_max = t_waypoints[-1]
    self.z_max = puntos_curva[-1, 2]

    self._N_tabla = 2000
    self._t_tabla = np.linspace(0, self.t_max, self._N_tabla)
    self._z_tabla = self.spline_z(self._t_tabla)

    def _z_a_t(self, z):
        """
        Convierte profundidad Z a parámetro t del spline.
        Usa interpolación lineal sobre la tabla precalculada.
        """
        if z <= self.P1[2]:
            # Tramo recto — mapeo lineal
            frac = z / (self.P1[2] + 1e-10)
            return 0.0
        else:
            # Tramo curvo — buscar en tabla
            idx = np.searchsorted(self._z_tabla, z)
            idx = np.clip(idx, 1, self._N_tabla - 1)
            z0 = self._z_tabla[idx - 1]
            z1 = self._z_tabla[idx]
            t0 = self._t_tabla[idx - 1]
            t1 = self._t_tabla[idx]
            frac = (z - z0) / (z1 - z0 + 1e-10)
            return t0 + frac * (t1 - t0)

    def evaluar(self, z):
        """
        Devuelve (x, y, z) de la trayectoria ideal a profundidad z.

        Args:
            z : profundidad [mm]

        Returns:
            array [x, y, z] en mm
        """
        if z <= 0:
            return np.array([0.0, 0.0, 0.0])

        if z <= self.L_bit:
            # Primeros L_bit mm: estrictamente vertical
            return np.array([0.0, 0.0, z])

        # Tramo curvo — spline (arranca en L_bit con tangente vertical)
        if True:
            # Tramo curvo — spline
            t = self._z_a_t(z)
            t = np.clip(t, 0, self.t_max)
            x = float(self.spline_x(t))
            y = float(self.spline_y(t))
            z_spline = float(self.spline_z(t))
            return np.array([x, y, z_spline])

    def direccion(self, z):
        """
        Devuelve el vector dirección tangente a la trayectoria en z.

        Args:
            z : profundidad [mm]

        Returns:
            vector unitario [dx, dy, dz]
        """
        if z <= self.L_bit:
            return np.array([0.0, 0.0, 1.0])
        else:
            # Tramo curvo — derivada del spline
            t = self._z_a_t(z)
            t = np.clip(t, 0, self.t_max)
            dx = float(self.spline_x(t, 1))
            dy = float(self.spline_y(t, 1))
            dz = float(self.spline_z(t, 1))
            tangente = np.array([dx, dy, dz])
            return tangente / (np.linalg.norm(tangente) + 1e-10)

    def curvatura(self, z):
        """
        Devuelve la curvatura 3D en la profundidad z.

        Returns:
            kappa [1/mm]
        """
        if z <= self.P1[2]:
            return 0.0

        t = self._z_a_t(z)
        t = np.clip(t, 0, self.t_max)

        # Primera y segunda derivada
        dx = self.spline_x(t, 1)
        dy = self.spline_y(t, 1)
        dz_dt = self.spline_z(t, 1)
        ddx = self.spline_x(t, 2)
        ddy = self.spline_y(t, 2)
        ddz = self.spline_z(t, 2)

        # Curvatura 3D: |r' x r''| / |r'|³
        cross = np.array([
            dy*ddz - dz_dt*ddy,
            dz_dt*ddx - dx*ddz,
            dx*ddy - dy*ddx
        ])
        num = np.linalg.norm(cross)
        den = (np.sqrt(dx**2 + dy**2 + dz_dt**2))**3

        return float(num / (den + 1e-12))


def generar_waypoints_random(rng=None, max_intentos=500):
    """
    Genera los 2 waypoints aleatorios que definen el tramo direccional
    (P2, P3), dentro del cono admisible. El primer waypoint (P1, fin
    del tramo recto) es fijo en (0,0,L_bit) y lo pone TrayectoriaIdeal.
    """
    if rng is None:
        rng = np.random.default_rng()

    Z2_range = (260, 400)
    Z3_range = (450, 550)
    XY_max = 100.0
    R_MIN = 186.43
    I_MAX = 30.0
    A_MAX = 15.0

    for _ in range(max_intentos):
        Z2 = rng.uniform(*Z2_range)
        Z3 = rng.uniform(*Z3_range)

        X2 = rng.uniform(0.3, 0.7) * XY_max
        X3 = rng.uniform(0.5, 1.0) * XY_max

        Y2 = rng.uniform(-0.25, 0.25) * XY_max
        Y3 = rng.uniform(-0.30, 0.30) * XY_max

        if np.sqrt(X3**2 + Y3**2) > XY_max:
            continue

        waypoints_curva = np.array([[X2, Y2, Z2], [X3, Y3, Z3]])

        try:
            tray = TrayectoriaIdeal(waypoints_curva=waypoints_curva)
        except Exception:
            continue

        zs = np.linspace(tray.L_bit + 1, tray.z_max - 1, 200)
        valida = True
        for z in zs:
            kappa = tray.curvatura(z)
            if kappa > 1e-9 and (1.0 / kappa) < R_MIN:
                valida = False
                break
            dx, dy, dz = tray.direccion(z)
            inclinacion = np.degrees(np.arccos(np.clip(dz, -1.0, 1.0)))
            azimut = np.degrees(np.arctan2(dy, dx))
            if inclinacion > I_MAX or abs(azimut) > A_MAX:
                valida = False
                break

        if valida:
            return waypoints_curva

    raise RuntimeError(f"No se encontró una trayectoria válida en {max_intentos} intentos.")