"""
validar_extrapolacion_UCS.py — Fase C: robustez fuera del rango de entrenamiento
===================================================================================
Mantiene (ROP, RPM) fijos en un par de combinaciones representativas, y
barre el UCS "real" de referencia desde bien por debajo hasta bien por
encima del rango de entrenamiento [20,30], derivando el WOB correcto con
la fórmula en cada paso (cada punto es físicamente consistente). Mide
cuánto se degrada la predicción de la red a medida que nos alejamos del
rango entrenado.

Pegar en NN_vert/ y correr: python validar_extrapolacion_UCS.py
"""
import numpy as np
import torch

from modelo import RedBHA
from datos import desnormalizar_y, normalizar_X
from config import A_TREPANO


def cargar_modelo(ruta="modelo_vert_WOB_RPM_ROP_UCS.pth"):
    checkpoint = torch.load(ruta, weights_only=False)
    modelo = RedBHA(capas=checkpoint['capas'])
    modelo.load_state_dict(checkpoint['model_state_dict'])
    modelo.eval()
    return modelo, checkpoint['stats']


def predecir_UCS(modelo, stats, rop, rpm, wob):
    X = np.array([[rop, rpm, wob]])
    X_tensor = normalizar_X(X, stats)
    with torch.no_grad():
        return desnormalizar_y(modelo(X_tensor), stats).item()


def wob_formula(rop, rpm, ucs, a=A_TREPANO):
    return np.pi * a * ucs * rop / (2 * rpm)


BASELINES = [("ROP=15,RPM=1500", 15.0, 1500.0), ("ROP=10,RPM=1200", 10.0, 1200.0)]
UCS_REF = np.arange(5, 47.5, 2.5)   # [20,30] es el rango de entrenamiento

if __name__ == "__main__":
    modelo, stats = cargar_modelo()

    for nombre, rop, rpm in BASELINES:
        print(f"\n{nombre}:")
        print(f"{'UCS_ref':>9} {'WOB':>9} {'UCS_red':>9} {'Desv.':>9} {'Desv.%':>9}  zona")
        for ucs_ref in UCS_REF:
            wob = wob_formula(rop, rpm, ucs_ref)
            ucs_red = predecir_UCS(modelo, stats, rop, rpm, wob)
            desv = ucs_red - ucs_ref
            desv_pct = desv / ucs_ref * 100
            zona = "DENTRO" if 20 <= ucs_ref <= 30 else "FUERA"
            print(f"{ucs_ref:9.2f} {wob:9.3f} {ucs_red:9.3f} {desv:+9.3f} {desv_pct:+8.2f}%  {zona}")