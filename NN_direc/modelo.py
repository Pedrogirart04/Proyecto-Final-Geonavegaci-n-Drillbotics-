

import torch.nn as nn
from config import ARQUITECTURA


class RedBHA(nn.Module):
    def __init__(self, capas=ARQUITECTURA):
        """
        Red densa con activación tanh en capas ocultas.
        Args:
            capas: lista con neuronas por capa.
                   Primera = n_inputs, última = n_outputs.
                   Ejemplo: [2, 64, 64, 64, 2]
        Cada capa oculta hace:
            z = W · input + b     (combinación lineal)
            a = tanh(z)           (activación no lineal)
        La última capa NO tiene activación para que la salida
        pueda ser cualquier valor real (positivo o negativo).
        """
        super().__init__()
        layers = []
        for i in range(len(capas) - 1):
            layers.append(nn.Linear(capas[i], capas[i+1]))
            # Activación tanh en todas las capas menos la última
            if i < len(capas) - 2:
                layers.append(nn.Tanh())
        self.red = nn.Sequential(*layers)

    def forward(self, x):
                return self.red(x)
