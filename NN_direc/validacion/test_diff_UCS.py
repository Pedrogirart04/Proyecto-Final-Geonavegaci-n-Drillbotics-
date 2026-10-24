from control import cargar_modelo
from optimizador import evaluar_red_roca

modelo, stats = cargar_modelo("modelo_bha.pth")
kappa_fijo = 0.000305  # el valor de clamp que vimos en todos los logs

for eps in [22, 24, 25, 26, 28]:
    F = evaluar_red_roca(modelo, stats, kappa_fijo, eps)
    print(f"ε={eps} MPa -> F_roca={F:.2f} N")