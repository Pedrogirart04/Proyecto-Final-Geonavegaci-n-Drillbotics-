"""
probar_red.py — Probar la red entrenada con inputs manuales
=============================================================
Script simple para explorar la relación F → curvatura.
Cargás el modelo entrenado y probás combinaciones de
fuerzas, WOB y RPM para ver qué curvatura predice la red.

Para correr: python probar_red.py
Requiere: modelo_bha.pth (generado por main.py)
"""

import torch
import numpy as np
from modelo import RedBHA
from datos import desnormalizar_y


def cargar_modelo(ruta="modelo_bha.pth"):
    checkpoint = torch.load(ruta, weights_only=False)
    modelo = RedBHA(capas=checkpoint['capas'])
    modelo.load_state_dict(checkpoint['model_state_dict'])
    modelo.eval()
    stats = checkpoint['stats']
    return modelo, stats


def predecir(modelo, stats, kappa, epsilon):
    X = np.array([[kappa, epsilon]])
    X_norm = (X - stats['X_min']) / (stats['X_max'] - stats['X_min'] + 1e-8)
    X_tensor = torch.tensor(X_norm, dtype=torch.float32)

    with torch.no_grad():
        y_norm = modelo(X_tensor)
        y_real = desnormalizar_y(y_norm, stats).numpy()[0]

    F_total = y_real[0]
    return F_total

# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":

    modelo, stats = cargar_modelo()
    print("Modelo cargado. Ingresá los valores para probar.")
    print("Escribí 'salir' para terminar.\n")

    while True:
        try:
            entrada = input("kappa [1/mm], epsilon [MPa]: ")

            if entrada.strip().lower() == 'salir':
                break

            valores = [float(v.strip()) for v in entrada.split(",")]
            kappa, epsilon = valores
            F_total = predecir(modelo, stats, kappa, epsilon)

            print(f"  F_total = {F_total:.2f} N")
            print()

        except ValueError:
            print("  Formato incorrecto. Ejemplo: 0.003, 25\n")
        except Exception as e:
            print(f"  Error: {e}\n")