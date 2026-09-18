"""
Red Neuronal — Relación Fuerzas → Curvatura (BHA)
==================================================
Aprende la relación entre las fuerzas de las patas del BHA
y la curvatura resultante del pozo.

Inputs:  F1, F2, F3 [N], WOB [N], RPM [rpm]
Outputs: dI/ds [°/mm], dA/ds [°/mm]

Estructura del archivo:
  1. Parámetros del sistema
  2. Generador de datos sintéticos  ← REEMPLAZAR con modelo del paper
  3. Dataset y normalización
  4. Arquitectura de la red
  5. Entrenamiento
  6. Evaluación
  7. Optimizador inverso (curvatura deseada → fuerzas)
  8. Main
"""

import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader, TensorDataset
import time

# ─────────────────────────────────────────────────────────────
# 1. PARÁMETROS DEL SISTEMA
# ─────────────────────────────────────────────────────────────

# Geometría de las patas [radianes]
THETA_PATAS = np.array([0, 2*np.pi/3, 4*np.pi/3])  # 0°, 120°, 240°

# Límites físicos de inputs
F_MAX   = 200.0   # Fuerza máxima por pata [N]
WOB_MIN = 5.0     # Weight on bit mínimo [N]
WOB_MAX = 50.0    # Weight on bit máximo [N]
RPM_MIN = 30.0    # RPM mínimas
RPM_MAX = 150.0   # RPM máximas

# Curvatura máxima admisible (de Rmin = 186.43 mm)
R_MIN      = 186.43   # [mm]
KAPPA_MAX  = 1.0 / R_MIN  # [1/mm] ≈ 5.36e-3
# Convertido a °/mm: kappa_max * (180/pi) ≈ 0.307 °/mm
KAPPA_MAX_DEG = KAPPA_MAX * (180.0 / np.pi)  # [°/mm]

# Coeficientes del modelo sintético
# ─────────────────────────────────────────────────────────────
# NOTA: estos coeficientes son PROVISORIOS.
# Reemplazar la función generar_datos() con el modelo del paper.
# ─────────────────────────────────────────────────────────────
ALPHA = 0.003      # Respuesta lineal principal [°/mm por N]
BETA  = 0.3        # Modulación de WOB (positivo = más WOB → más curvatura)
GAMMA = -0.2       # Modulación de RPM (negativo = más RPM → menos curvatura)
DELTA = 5e-6       # Término no lineal cuadrático
SIGMA_RUIDO = 0.005  # Desviación estándar del ruido gaussiano [°/mm]


# ─────────────────────────────────────────────────────────────
# 2. GENERADOR DE DATOS SINTÉTICOS
#    ↓↓↓ REEMPLAZAR ESTA FUNCIÓN CON EL MODELO DEL PAPER ↓↓↓
# ─────────────────────────────────────────────────────────────

def generar_datos(N_puntos=15000, semilla=42):
    """
    Genera un dataset sintético de N_puntos muestreando aleatoriamente
    el espacio de inputs y calculando outputs con el modelo simplificado.

    Returns:
        X : array (N_válidos, 5) — inputs  [F1, F2, F3, WOB, RPM]
        y : array (N_válidos, 2) — outputs [dI/ds, dA/ds]

    ─────────────────────────────────────────────────────────
    PARA REEMPLAZAR CON EL MODELO DEL PAPER:
    
    La función debe devolver el mismo formato (X, y).
    Los inputs X siempre tienen columnas [F1, F2, F3, WOB, RPM].
    Los outputs y siempre tienen columnas [dI/ds, dA/ds] en °/mm.
    
    El filtro de curvatura máxima al final de esta función
    se puede mantener o adaptar según corresponda.
    ─────────────────────────────────────────────────────────
    """
    rng = np.random.default_rng(semilla)

    # Muestreo aleatorio uniforme del espacio de inputs
    F1  = rng.uniform(0, F_MAX,   N_puntos)
    F2  = rng.uniform(0, F_MAX,   N_puntos)
    F3  = rng.uniform(0, F_MAX,   N_puntos)
    WOB = rng.uniform(WOB_MIN, WOB_MAX, N_puntos)
    RPM = rng.uniform(RPM_MIN, RPM_MAX, N_puntos)

    # Fuerza lateral resultante en x e y
    # (proyección de cada pata según su ángulo)
    Fx = (F1 * np.cos(THETA_PATAS[0]) +
          F2 * np.cos(THETA_PATAS[1]) +
          F3 * np.cos(THETA_PATAS[2]))

    Fy = (F1 * np.sin(THETA_PATAS[0]) +
          F2 * np.sin(THETA_PATAS[1]) +
          F3 * np.sin(THETA_PATAS[2]))

    # Factor de modulación por WOB y RPM
    mod = (1 + BETA  * (WOB - WOB_MIN) / (WOB_MAX - WOB_MIN)) * \
          (1 + GAMMA * (RPM - RPM_MIN) / (RPM_MAX - RPM_MIN))

    # Curvatura con término lineal + no lineal + ruido
    ruido_I = rng.normal(0, SIGMA_RUIDO, N_puntos)
    ruido_A = rng.normal(0, SIGMA_RUIDO, N_puntos)

    dIds = ALPHA * Fx * mod + DELTA * Fx**2 + ruido_I
    dAds = ALPHA * Fy * mod + DELTA * Fy**2 + ruido_A

    # Filtrar puntos que superan la curvatura máxima física
    kappa = np.sqrt(dIds**2 + dAds**2)
    mascara = kappa <= KAPPA_MAX_DEG

    n_validos   = mascara.sum()
    n_descartados = N_puntos - n_validos
    print(f"Puntos generados:   {N_puntos}")
    print(f"Puntos válidos:     {n_validos} ({100*n_validos/N_puntos:.1f}%)")
    print(f"Puntos descartados: {n_descartados} (superan κ_max)")

    X = np.column_stack([F1, F2, F3, WOB, RPM])[mascara]
    y = np.column_stack([dIds, dAds])[mascara]

    return X, y


# ─────────────────────────────────────────────────────────────
# 3. DATASET Y NORMALIZACIÓN
# ─────────────────────────────────────────────────────────────

def preparar_dataset(X, y, fraccion_val=0.2):
    """
    Normaliza inputs y outputs a [0,1] y divide en train/val.

    La normalización es importante: la red aprende mejor cuando
    todas las variables tienen magnitudes similares. Sin esto,
    F [0-200 N] dominaría sobre RPM [30-150 rpm] en los gradientes.

    Returns:
        X_train, y_train, X_val, y_val : tensores de PyTorch
        stats : diccionario con min/max para desnormalizar después
    """
    # Guardar estadísticas para desnormalizar después
    X_min = X.min(axis=0)
    X_max = X.max(axis=0)
    y_min = y.min(axis=0)
    y_max = y.max(axis=0)

    stats = {
        'X_min': X_min, 'X_max': X_max,
        'y_min': y_min, 'y_max': y_max
    }

    # Normalización min-max a [0, 1]
    X_norm = (X - X_min) / (X_max - X_min + 1e-8)
    y_norm = (y - y_min) / (y_max - y_min + 1e-8)

    # División train/validación
    N = len(X_norm)
    N_val   = int(N * fraccion_val)
    N_train = N - N_val

    idx = np.random.permutation(N)
    idx_train = idx[:N_train]
    idx_val   = idx[N_train:]

    X_train = torch.tensor(X_norm[idx_train], dtype=torch.float32)
    y_train = torch.tensor(y_norm[idx_train], dtype=torch.float32)
    X_val   = torch.tensor(X_norm[idx_val],   dtype=torch.float32)
    y_val   = torch.tensor(y_norm[idx_val],   dtype=torch.float32)

    print(f"\nDataset:")
    print(f"  Train: {N_train} puntos")
    print(f"  Val:   {N_val} puntos")

    return X_train, y_train, X_val, y_val, stats


def desnormalizar_y(y_norm, stats):
    """Convierte outputs normalizados de vuelta a °/mm."""
    y_min = torch.tensor(stats['y_min'], dtype=torch.float32)
    y_max = torch.tensor(stats['y_max'], dtype=torch.float32)
    return y_norm * (y_max - y_min) + y_min


def normalizar_X(X_np, stats):
    """Normaliza un array de inputs para usar con el modelo entrenado."""
    X_norm = (X_np - stats['X_min']) / (stats['X_max'] - stats['X_min'] + 1e-8)
    return torch.tensor(X_norm, dtype=torch.float32)


# ─────────────────────────────────────────────────────────────
# 4. ARQUITECTURA DE LA RED
# ─────────────────────────────────────────────────────────────

class RedBHA(nn.Module):
    def __init__(self, capas=[5, 64, 64, 64, 2]):
        """
        Red densa con activación tanh.

        capas: lista con neuronas por capa.
               Primera = n_inputs (5), Última = n_outputs (2).

        Diferencia respecto a la viga:
          - 5 inputs en vez de 1
          - 2 outputs en vez de 1
          - Más neuronas por capa (64 vs 32) por mayor complejidad
          - Es supervisada: aprende de datos, no de residuo de EDE
        """
        super().__init__()

        layers = []
        for i in range(len(capas) - 1):
            layers.append(nn.Linear(capas[i], capas[i+1]))
            if i < len(capas) - 2:
                layers.append(nn.Tanh())

        self.red = nn.Sequential(*layers)

    def forward(self, x):
        return self.red(x)


# ─────────────────────────────────────────────────────────────
# 5. ENTRENAMIENTO
# ─────────────────────────────────────────────────────────────

def entrenar(modelo, X_train, y_train, X_val, y_val,
             N_epochs=3000, lr=1e-3, batch_size=256):
    """
    Entrenamiento supervisado con Adam + L-BFGS.

    Diferencias respecto a la viga:
      - Loss = MSE entre predicción y output real (no residuo de EDE)
      - Se usa mini-batches (batch_size) porque el dataset es grande
      - Se monitorea la loss de validación para detectar overfitting
      - Best checkpoint basado en loss de validación
    """
    criterio    = nn.MSELoss()
    optimizador = torch.optim.Adam(modelo.parameters(), lr=lr)

    dataset_train = TensorDataset(X_train, y_train)
    loader_train  = DataLoader(dataset_train, batch_size=batch_size, shuffle=True)

    historial_train = []
    historial_val   = []

    mejor_val_loss  = float('inf')
    mejor_estado    = None

    print("\n" + "─" * 65)
    print("FASE 1 — Adam")
    print("─" * 65)

    for epoch in range(N_epochs):
        # ── Training ──────────────────────────────────────────────
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

        # ── Validación ────────────────────────────────────────────
        modelo.eval()
        with torch.no_grad():
            y_val_pred  = modelo(X_val)
            loss_val    = criterio(y_val_pred, y_val).item()

        historial_train.append(loss_train_epoch)
        historial_val.append(loss_val)

        # Best checkpoint basado en validación
        if loss_val < mejor_val_loss:
            mejor_val_loss = loss_val
            mejor_estado   = {k: v.clone() for k, v in modelo.state_dict().items()}

        if epoch % 500 == 0 or epoch == N_epochs - 1:
            print(f"Epoch {epoch:5d} | Loss train: {loss_train_epoch:.4e} "
                  f"| Loss val: {loss_val:.4e}")

    modelo.load_state_dict(mejor_estado)
    print(f"\nBest checkpoint restaurado — mejor val loss: {mejor_val_loss:.4e}")

    # ── Fase L-BFGS ───────────────────────────────────────────────
    print("\n" + "─" * 65)
    print("FASE 2 — L-BFGS")
    print("─" * 65)

    optimizador_lbfgs = torch.optim.LBFGS(
        modelo.parameters(),
        lr=1.0,
        max_iter=20,
        history_size=50,
        line_search_fn='strong_wolfe'
    )

    mejor_val_loss_lbfgs = mejor_val_loss
    mejor_estado_lbfgs   = {k: v.clone() for k, v in modelo.state_dict().items()}

    for iteracion in range(200):

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

        if iteracion % 50 == 0 or iteracion == 199:
            print(f"LBFGS {iteracion:5d} | Loss val: {loss_val:.4e}")

    modelo.load_state_dict(mejor_estado_lbfgs)
    print(f"\nBest checkpoint restaurado — mejor val loss L-BFGS: {mejor_val_loss_lbfgs:.4e}")

    return historial_train, historial_val


# ─────────────────────────────────────────────────────────────
# 6. EVALUACIÓN
# ─────────────────────────────────────────────────────────────

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

    error_dIds = np.abs(y_pred_real[:, 0] - y_real[:, 0])
    error_dAds = np.abs(y_pred_real[:, 1] - y_real[:, 1])

    print(f"\n{'─'*65}")
    print(f"EVALUACIÓN EN VALIDACIÓN")
    print(f"{'─'*65}")
    print(f"dI/ds — Error medio: {error_dIds.mean():.4e} °/mm | "
          f"Error max: {error_dIds.max():.4e} °/mm")
    print(f"dA/ds — Error medio: {error_dAds.mean():.4e} °/mm | "
          f"Error max: {error_dAds.max():.4e} °/mm")

    # Error relativo respecto al rango
    rango_dIds = np.abs(y_real[:, 0]).max() + 1e-10
    rango_dAds = np.abs(y_real[:, 1]).max() + 1e-10
    err_rel_dIds = error_dIds / rango_dIds * 100
    err_rel_dAds = error_dAds / rango_dAds * 100
    print(f"dI/ds — Error relativo máximo: {err_rel_dIds.max():.2f}%")
    print(f"dA/ds — Error relativo máximo: {err_rel_dAds.max():.2f}%")

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    fig.suptitle("Evaluación Red Neuronal BHA", fontsize=14)

    # — dI/ds: predicho vs real —
    axes[0,0].scatter(y_real[:,0], y_pred_real[:,0], alpha=0.3, s=5)
    lim = max(np.abs(y_real[:,0]).max(), np.abs(y_pred_real[:,0]).max())
    axes[0,0].plot([-lim, lim], [-lim, lim], 'r--', lw=1.5, label="ideal")
    axes[0,0].set_xlabel("dI/ds real [°/mm]")
    axes[0,0].set_ylabel("dI/ds predicho [°/mm]")
    axes[0,0].set_title("dI/ds — Predicho vs Real")
    axes[0,0].legend()
    axes[0,0].grid(True)

    # — dA/ds: predicho vs real —
    axes[1,0].scatter(y_real[:,1], y_pred_real[:,1], alpha=0.3, s=5)
    lim = max(np.abs(y_real[:,1]).max(), np.abs(y_pred_real[:,1]).max())
    axes[1,0].plot([-lim, lim], [-lim, lim], 'r--', lw=1.5, label="ideal")
    axes[1,0].set_xlabel("dA/ds real [°/mm]")
    axes[1,0].set_ylabel("dA/ds predicho [°/mm]")
    axes[1,0].set_title("dA/ds — Predicho vs Real")
    axes[1,0].legend()
    axes[1,0].grid(True)

    # — Histograma error dI/ds —
    axes[0,1].hist(error_dIds, bins=50, color='steelblue', edgecolor='white')
    axes[0,1].set_xlabel("Error absoluto [°/mm]")
    axes[0,1].set_title("Distribución error dI/ds")
    axes[0,1].grid(True)

    # — Histograma error dA/ds —
    axes[1,1].hist(error_dAds, bins=50, color='steelblue', edgecolor='white')
    axes[1,1].set_xlabel("Error absoluto [°/mm]")
    axes[1,1].set_title("Distribución error dA/ds")
    axes[1,1].grid(True)

    # — Curva de convergencia —
    N_adam  = len(historial_train)
    N_lbfgs = len(historial_val) - N_adam
    epochs_adam  = np.arange(N_adam)
    epochs_lbfgs = np.arange(N_adam, N_adam + N_lbfgs)

    axes[0,2].semilogy(epochs_adam, historial_train,
                       label="Train (Adam)", color="steelblue")
    axes[0,2].semilogy(epochs_adam, historial_val[:N_adam],
                       label="Val (Adam)", color="orange")
    if N_lbfgs > 0:
        axes[0,2].semilogy(epochs_lbfgs, historial_val[N_adam:],
                           label="Val (L-BFGS)", color="green")
    axes[0,2].set_xlabel("Epoch / Iteración")
    axes[0,2].set_ylabel("Loss MSE (log)")
    axes[0,2].set_title("Convergencia")
    axes[0,2].legend()
    axes[0,2].grid(True)

    # — Error relativo por punto de validación —
    axes[1,2].plot(err_rel_dIds, alpha=0.5, label="dI/ds", lw=0.5)
    axes[1,2].plot(err_rel_dAds, alpha=0.5, label="dA/ds", lw=0.5)
    axes[1,2].set_xlabel("Punto de validación")
    axes[1,2].set_ylabel("Error relativo [%]")
    axes[1,2].set_title("Error relativo por punto")
    axes[1,2].legend()
    axes[1,2].grid(True)

    plt.tight_layout()
    plt.savefig("resultado_nn_bha.png", dpi=150)
    plt.show()


# ─────────────────────────────────────────────────────────────
# 7. OPTIMIZADOR INVERSO
#    Dado (dI/ds*, dA/ds*) deseados → encontrar (F1, F2, F3)
# ─────────────────────────────────────────────────────────────

def optimizador_inverso(modelo, stats,
                        dIds_deseado, dAds_deseado,
                        WOB_actual, RPM_actual,
                        n_iter=200, lr=0.01):
    """
    Dado una curvatura deseada (dI/ds*, dA/ds*), encuentra la
    combinación óptima de fuerzas (F1, F2, F3) que la produce.

    Estrategia: gradiente proyectado sobre F1,F2,F3 ∈ [0, F_MAX]
    minimizando || NN(F1,F2,F3,WOB,RPM) - (dI/ds*, dA/ds*) ||²
    con penalización de mínima norma de fuerzas.

    Args:
        dIds_deseado, dAds_deseado : curvatura objetivo [°/mm]
        WOB_actual, RPM_actual     : condiciones actuales de perforación
        n_iter                     : iteraciones del optimizador
        lr                         : learning rate del optimizador inverso

    Returns:
        F_opt : array [F1, F2, F3] en Newtons
        error : error final entre curvatura predicha y deseada
    """
    modelo.eval()

    # Normalizar WOB y RPM actuales
    WOB_n = (WOB_actual - stats['X_min'][3]) / (stats['X_max'][3] - stats['X_min'][3] + 1e-8)
    RPM_n = (RPM_actual - stats['X_min'][4]) / (stats['X_max'][4] - stats['X_min'][4] + 1e-8)

    # Target normalizado
    y_min = torch.tensor(stats['y_min'], dtype=torch.float32)
    y_max = torch.tensor(stats['y_max'], dtype=torch.float32)
    target = torch.tensor(
        [(dIds_deseado - stats['y_min'][0]) / (stats['y_max'][0] - stats['y_min'][0] + 1e-8),
         (dAds_deseado - stats['y_min'][1]) / (stats['y_max'][1] - stats['y_min'][1] + 1e-8)],
        dtype=torch.float32
    )

    # Inicializar fuerzas en el centro del rango (normalizado = 0.5)
    F_norm = torch.tensor([0.5, 0.5, 0.5], dtype=torch.float32, requires_grad=True)
    opt    = torch.optim.Adam([F_norm], lr=lr)

    lambda_norma = 0.01  # Peso de penalización mínima norma

    for _ in range(n_iter):
        opt.zero_grad()

        # Clamp para mantener F en [0, 1] normalizado (proyección)
        F_clamped = torch.clamp(F_norm, 0.0, 1.0)

        # Armar input completo para la red
        x_input = torch.cat([
            F_clamped,
            torch.tensor([WOB_n, RPM_n], dtype=torch.float32)
        ]).unsqueeze(0)

        y_pred = modelo(x_input).squeeze()

        # Loss = error curvatura + penalización norma mínima de fuerzas
        loss_curvatura = torch.mean((y_pred - target)**2)
        loss_norma     = lambda_norma * torch.mean(F_clamped**2)
        loss           = loss_curvatura + loss_norma

        loss.backward()
        opt.step()

    # Fuerzas finales desnormalizadas
    with torch.no_grad():
        F_final   = torch.clamp(F_norm, 0.0, 1.0).numpy()
        F_opt     = F_final * F_MAX

        # Verificar curvatura máxima
        x_final   = torch.tensor(
            np.concatenate([F_final, [WOB_n, RPM_n]]),
            dtype=torch.float32
        ).unsqueeze(0)
        y_final   = modelo(x_final).squeeze()
        curv_pred = desnormalizar_y(y_final.unsqueeze(0), stats).numpy()[0]
        kappa     = np.sqrt(curv_pred[0]**2 + curv_pred[1]**2)

        if kappa > KAPPA_MAX_DEG:
            print(f"  ADVERTENCIA: curvatura predicha ({kappa:.4f} °/mm) "
                  f"supera κ_max ({KAPPA_MAX_DEG:.4f} °/mm)")

    error = np.sqrt((curv_pred[0] - dIds_deseado)**2 +
                    (curv_pred[1] - dAds_deseado)**2)

    return F_opt, error, curv_pred


# ─────────────────────────────────────────────────────────────
# 8. MAIN
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":

    torch.manual_seed(42)
    np.random.seed(42)

    # ── Generar datos ─────────────────────────────────────────
    print("=" * 65)
    print("GENERANDO DATOS SINTÉTICOS")
    print("=" * 65)
    X, y = generar_datos(N_puntos=15000)

    # ── Preparar dataset ──────────────────────────────────────
    X_train, y_train, X_val, y_val, stats = preparar_dataset(X, y)

    # ── Crear y entrenar la red ───────────────────────────────
    print("\n" + "=" * 65)
    print("ENTRENAMIENTO")
    print("=" * 65)
    modelo = RedBHA(capas=[5, 64, 64, 64, 2])

    start = time.time()
    historial_train, historial_val = entrenar(
        modelo, X_train, y_train, X_val, y_val,
        N_epochs=3000, lr=1e-3, batch_size=256
    )
    elapsed = time.time() - start
    print(f"\nTiempo de entrenamiento: {elapsed:.2f} segundos")

    # ── Evaluar ───────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("EVALUACIÓN")
    print("=" * 65)
    evaluar(modelo, X_val, y_val, stats, historial_train, historial_val)

    # ── Ejemplo de uso del optimizador inverso ────────────────
    print("\n" + "=" * 65)
    print("EJEMPLO OPTIMIZADOR INVERSO")
    print("=" * 65)
    dIds_objetivo = 0.10   # °/mm — curvatura deseada en inclinación
    dAds_objetivo = 0.05   # °/mm — curvatura deseada en azimut
    WOB_now       = 25.0   # N
    RPM_now       = 80.0   # rpm

    print(f"Curvatura deseada: dI/ds = {dIds_objetivo} °/mm, "
          f"dA/ds = {dAds_objetivo} °/mm")
    print(f"Condiciones: WOB = {WOB_now} N, RPM = {RPM_now} rpm")

    F_opt, error, curv_pred = optimizador_inverso(
        modelo, stats,
        dIds_objetivo, dAds_objetivo,
        WOB_now, RPM_now
    )

    print(f"\nFuerzas óptimas encontradas:")
    print(f"  F1 (  0°): {F_opt[0]:.2f} N")
    print(f"  F2 (120°): {F_opt[1]:.2f} N")
    print(f"  F3 (240°): {F_opt[2]:.2f} N")
    print(f"\nCurvatura predicha con esas fuerzas:")
    print(f"  dI/ds = {curv_pred[0]:.4f} °/mm  (objetivo: {dIds_objetivo})")
    print(f"  dA/ds = {curv_pred[1]:.4f} °/mm  (objetivo: {dAds_objetivo})")
    print(f"  Error: {error:.4e} °/mm")

    # ── Guardar modelo ────────────────────────────────────────
    torch.save({
        'model_state_dict': modelo.state_dict(),
        'stats': stats,
        'capas': [5, 64, 64, 64, 2]
    }, "modelo_bha.pth")
    print("\nModelo guardado en: modelo_bha.pth")



Esos 12000 puntos que elegis, que son? osea cada iteracion en vez de comparar 200 puntos compara 12000 pero no entiendo que representan, son puntos fisicos osea de prueba de como seria la trayectoria o que son? 

Por otro lado, me gustaria seguir la estrategia de varias funciones en archivos separados y un main, se puede implementar facilmente eso?