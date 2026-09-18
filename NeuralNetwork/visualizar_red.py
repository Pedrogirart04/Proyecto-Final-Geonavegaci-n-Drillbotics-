import numpy as np
import torch
import matplotlib.pyplot as plt
from modelo import RedBHA
from datos import desnormalizar_y

# Cargar modelo
checkpoint = torch.load("modelo_bha.pth", weights_only=False)
modelo = RedBHA(capas=checkpoint['capas'])
modelo.load_state_dict(checkpoint['model_state_dict'])
modelo.eval()
stats = checkpoint['stats']

# Barrido de kappa
kappas = np.linspace(0.001, 0.0053, 200)

# Curva para cada epsilon
fig, ax = plt.subplots(figsize=(10, 6))

for eps in [22, 23, 24, 25, 26, 27, 28]:
    F_pred = []
    for k in kappas:
        k_norm = (k - stats['X_min'][0]) / (stats['X_max'][0] - stats['X_min'][0] + 1e-8)
        e_norm = (eps - stats['X_min'][1]) / (stats['X_max'][1] - stats['X_min'][1] + 1e-8)
        x_input = torch.tensor([[k_norm, e_norm]], dtype=torch.float32)
        with torch.no_grad():
            y_norm = modelo(x_input)
            F = desnormalizar_y(y_norm, stats).item()
        F_pred.append(F)
    ax.plot(kappas * 1000, F_pred, label=f'ε = {eps} MPa')

ax.set_xlabel('κ [1/mm]')
ax.set_ylabel('F_roca [N]')
ax.set_title('Relación κ → F_roca aprendida por la red para distintos UCS')
ax.legend()
ax.grid(True)
plt.savefig('relacion_red_aprendida.png', dpi=150)
plt.show()