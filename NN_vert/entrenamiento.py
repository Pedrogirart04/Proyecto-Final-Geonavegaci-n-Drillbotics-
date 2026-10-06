"""
entrenamiento.py — Entrenamiento y evaluación de la red
========================================================
Contiene:
  - entrenar() : loop Adam + L-BFGS con best checkpoint
  - evaluar()  : métricas y gráficos de evaluación
"""
import time
import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader, TensorDataset
from datos import desnormalizar_y
from config import N_EPOCHS, LR_ADAM, BATCH_SIZE, N_LBFGS


def entrenar(modelo, X_train, y_train, X_val, y_val,
             N_epochs=N_EPOCHS, lr=LR_ADAM, batch_size=BATCH_SIZE):
    """
    Entrenamiento supervisado en dos fases: Adam + L-BFGS.

      - Loss = MSE entre predicción y output real 
      - Se usan mini-batches porque el dataset es grande (~12000 puntos)
      - Se monitorea loss de validación para detectar overfitting
      - Best checkpoint basado en loss de validación, no de training

    Mini-batches: en vez de usar los 12000 puntos por epoch,
    se dividen en grupos de 256. En cada epoch la red ve todos
    los puntos pero en grupos — esto acelera el entrenamiento
    y agrega ruido al gradiente que ayuda a escapar mínimos locales.

    Args:
        modelo   : instancia de RedBHA
        X_train, y_train : datos de entrenamiento normalizados
        X_val, y_val     : datos de validación normalizados
        N_epochs : epochs de Adam
        lr       : learning rate de Adam
        batch_size : tamaño de mini-batch

    Returns:
        historial_train : loss de training por epoch (Adam)
        historial_val   : loss de validación por epoch (Adam + L-BFGS)
    """
    criterio    = nn.MSELoss()
    optimizador = torch.optim.Adam(modelo.parameters(), lr=lr)

    dataset_train = TensorDataset(X_train, y_train)
    loader_train  = DataLoader(dataset_train, batch_size=batch_size, shuffle=True)

    historial_train = []
    historial_val   = []

    mejor_val_loss = float('inf')
    mejor_estado   = None

    # ══════════════════════════════════════════════════════════
    # FASE 1 — Adam
    # ══════════════════════════════════════════════════════════
    print("\n" + "─" * 65)
    print("FASE 1 — Adam")
    print("─" * 65)
    t_adam = time.time()
    for epoch in range(N_epochs):
        # ── Training ──────────────────────────────────────────
        modelo.train()
        loss_train_epoch = 0.0

        for X_batch, y_batch in loader_train:
            optimizador.zero_grad()
            y_pred = modelo(X_batch)
            loss   = criterio(y_pred, y_batch)
            loss.backward()
            optimizador.step()
            loss_train_epoch += loss.item() * len(X_batch)

        loss_train_epoch /= len(X_train)

        # ── Validación ────────────────────────────────────────
        modelo.eval()
        with torch.no_grad():
            y_val_pred = modelo(X_val)
            loss_val   = criterio(y_val_pred, y_val).item()

        historial_train.append(loss_train_epoch)
        historial_val.append(loss_val)

        # Best checkpoint basado en validación
        if loss_val < mejor_val_loss:
            mejor_val_loss = loss_val
            mejor_estado   = {k: v.clone() for k, v in modelo.state_dict().items()}

        if epoch % 100 == 0 or epoch == N_epochs - 1:
            print(f"Epoch {epoch:5d} | Loss train: {loss_train_epoch:.4e} "
                  f"| Loss val: {loss_val:.4e}")

    modelo.load_state_dict(mejor_estado)
    print(f"\nBest checkpoint restaurado — mejor val loss: {mejor_val_loss:.4e}")

    print(f"Tiempo Adam: {time.time() - t_adam:.2f}s")
    
    
    # ══════════════════════════════════════════════════════════
    # FASE 2 — L-BFGS
    # ══════════════════════════════════════════════════════════
    print("\n" + "─" * 65)
    print("FASE 2 — L-BFGS")
    print("─" * 65)
    t_lbfgs = time.time()
    
    optimizador_lbfgs = torch.optim.LBFGS(
        modelo.parameters(),
        lr=1.0,
        max_iter=20,
        history_size=50,
        line_search_fn='strong_wolfe'
    )

    mejor_val_loss_lbfgs = mejor_val_loss
    mejor_estado_lbfgs   = {k: v.clone() for k, v in modelo.state_dict().items()}

    for iteracion in range(N_LBFGS):

        def closure():
            optimizador_lbfgs.zero_grad()
            y_pred = modelo(X_train)
            loss   = criterio(y_pred, y_train)
            loss.backward()
            return loss

        optimizador_lbfgs.step(closure)

        modelo.eval()
        with torch.no_grad():
            loss_val = criterio(modelo(X_val), y_val).item()

        historial_val.append(loss_val)

        if loss_val < mejor_val_loss_lbfgs:
            mejor_val_loss_lbfgs = loss_val
            mejor_estado_lbfgs   = {k: v.clone() for k, v in modelo.state_dict().items()}

        if iteracion % 20 == 0 or iteracion == N_LBFGS - 1:
            print(f"LBFGS {iteracion:5d} | Loss val: {loss_val:.4e}")

        modelo.load_state_dict(mejor_estado_lbfgs)
    print(f"\nBest checkpoint restaurado — mejor val loss L-BFGS: {mejor_val_loss_lbfgs:.4e}")
    print(f"Tiempo L-BFGS: {time.time() - t_lbfgs:.2f}s")

    return historial_train, historial_val

def evaluar(modelo, X_val, y_val, stats, historial_train, historial_val):
    """
    Evalúa la red en el conjunto de validación y grafica resultados.
    """
    modelo.eval()
    with torch.no_grad():
        y_pred_norm = modelo(X_val)

    # Desnormalizar para comparar en unidades reales
    y_pred_real = desnormalizar_y(y_pred_norm, stats).numpy()
    y_real      = desnormalizar_y(y_val,       stats).numpy()

    error = np.abs(y_pred_real[:, 0] - y_real[:, 0])

    print(f"\n{'─'*65}")
    print(f"EVALUACIÓN EN VALIDACIÓN")
    print(f"{'─'*65}")
    print(f"F_roca — Error medio: {error.mean():.4f} Mpa | "
          f"Error max: {error.max():.4f} Mpa")

    # Error relativo respecto al rango
    rango = np.abs(y_real[:, 0]).max() + 1e-10
    err_rel = error / rango * 100
    print(f"F_roca — Error relativo máximo: {err_rel.max():.2f}%")

    # ── Gráficos ──────────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Evaluación Red Neuronal BHA", fontsize=14)

    # — F_roca: predicho vs real —
    axes[0].scatter(y_real[:,0], y_pred_real[:,0], alpha=0.3, s=5)
    lim = max(np.abs(y_real[:,0]).max(), np.abs(y_pred_real[:,0]).max())
    axes[0].plot([0, lim], [0, lim], 'r--', lw=1.5, label="ideal")
    axes[0].set_xlabel("F_roca real [N]")
    axes[0].set_ylabel("F_roca predicho [N]")
    axes[0].set_title("Predicho vs Real")
    axes[0].legend()
    axes[0].grid(True)

    # — Histograma error —
    axes[1].hist(error, bins=50, color='steelblue', edgecolor='white')
    axes[1].set_xlabel("Error absoluto [N]")
    axes[1].set_ylabel("Cantidad de puntos")
    axes[1].set_title("Distribución del error")
    axes[1].grid(True)

    # — Curva de convergencia —
    N_adam  = len(historial_train)
    N_lbfgs = len(historial_val) - N_adam
    epochs_adam  = np.arange(N_adam)
    epochs_lbfgs = np.arange(N_adam, N_adam + N_lbfgs)

    axes[2].semilogy(epochs_adam, historial_train,
                     label="Train (Adam)", color="steelblue")
    axes[2].semilogy(epochs_adam, historial_val[:N_adam],
                     label="Val (Adam)", color="orange")
    if N_lbfgs > 0:
        axes[2].semilogy(epochs_lbfgs, historial_val[N_adam:],
                         label="Val (L-BFGS)", color="green")
    axes[2].set_xlabel("Epoch / Iteración")
    axes[2].set_ylabel("Loss MSE (log)")
    axes[2].set_title("Convergencia")
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig("resultado_nn_bha.png", dpi=150)
    plt.show()
