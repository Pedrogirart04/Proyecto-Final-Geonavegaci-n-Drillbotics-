%% =========================================================================
% GENERAR_DATASET_ROP - Dataset con ROP variable para simulador
% =========================================================================
% Genera pares (kappa, ROP) → F_roca variando el ROP en cada trayectoria.
% Epsilon se mantiene fijo en 25 MPa.
%
% Salida: CSV con columnas:
%   F_roca_N | kappa_1_mm | ROP_mm_min | R_mm | Z_mm
% =========================================================================
clear; clc; close all;

%% 1. PARÁMETROS DE GENERACIÓN
N_trayectorias = 50;  % <-- Subir para dataset final
semilla = 42;
rng(semilla);

%% 2. PARÁMETROS FIJOS DEL SISTEMA

% Roca — epsilon FIJO
params_roca.epsilon = 25;
params_roca.zeta_f  = 1;
params_roca.zeta_g  = 10;

% Trépano
params_trepano.a = 1.5 * 25.4 / 2;
params_trepano.b = 5;
params_trepano.L_bit = 88;

% Sarta
col_deflexion = 1;
col_z = 3;

% Cables y palanca
g = 9.81;
l_a = 7.27;  l_b = 4.11;
d_a = 10.8;  d_b = 13.88;
coef_I = [1.55957502e-04, 3.33106612e-03, 2.37648267e-02, 5.90535677e-02];
R_min = 186.43;
R_umbral = 1000;

% RPM fijo
RPM = 1500;

% Rango de ROP a explorar
ROP_MIN = 5;    % [mm/min]
ROP_MAX = 20;   % [mm/min]

%% 3. LÍMITES DEL CONO ADMISIBLE
Z1_range = [150, 220];
Z2_range = [250, 400];
Z3_range = [450, 580];
XY_max = 100;

%% 4. ACUMULADOR DE DATOS
% Columnas: F_roca | kappa | ROP | R | Z
datos_acumulados = [];

%% 5. LOOP DE GENERACIÓN
fprintf('=========================================\n');
fprintf('  GENERACIÓN DE DATASET (ROP variable)\n');
fprintf('=========================================\n');

trayectorias_validas = 0;
intentos = 0;
max_intentos = N_trayectorias * 5;

while trayectorias_validas < N_trayectorias && intentos < max_intentos
    intentos = intentos + 1;

    %% 5.1 Generar waypoints aleatorios
    Z1 = Z1_range(1) + rand() * (Z1_range(2) - Z1_range(1));
    Z2 = Z2_range(1) + rand() * (Z2_range(2) - Z2_range(1));
    Z3 = Z3_range(1) + rand() * (Z3_range(2) - Z3_range(1));

    X1 = (rand() * 0.3) * XY_max;
    X2 = (0.3 + rand() * 0.4) * XY_max;
    X3 = (0.5 + rand() * 0.5) * XY_max;

    Y1 = (rand() - 0.5) * 0.3 * XY_max;
    Y2 = (rand() - 0.5) * 0.5 * XY_max;
    Y3 = (rand() - 0.5) * 0.6 * XY_max;

    desp_horiz = sqrt(X3^2 + Y3^2);
    if desp_horiz > XY_max
        continue;
    end

    P1 = [X1, Y1, Z1];
    P2 = [X2, Y2, Z2];
    P3 = [X3, Y3, Z3];

    %% 5.2 Sortear ROP para esta trayectoria
    ROP_mm_min = ROP_MIN + rand() * (ROP_MAX - ROP_MIN);

    %% 5.3 Generar trayectoria
    try
        Matriz_Trayectoria = generar_trayectoria_silent(P1, P2, P3);
    catch
        fprintf('  [!] Trayectoria %d falló en generación, descartada.\n', intentos);
        continue;
    end

    R_tray = Matriz_Trayectoria(:, 4);
    R_min_real = min(R_tray(R_tray > 0 & R_tray < 10000));
    if isempty(R_min_real) || R_min_real < R_min
        fprintf('  [!] Trayectoria %d viola R_min (R=%.1f mm), descartada.\n', intentos, R_min_real);
        continue;
    end

    %% 5.4 Calcular fuerzas
    try
        Matriz_Fuerzas = calcular_fuerzas_Perneder_silent(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano);
        F2 = Matriz_Fuerzas(:, 2);
        F3 = Matriz_Fuerzas(:, 3);

        Matriz_fuerzas_cables = calcular_fuerza_actuacion_silent(Matriz_Trayectoria, F2, F3, R_min, R_umbral, l_a, l_b, d_a, d_b, g, coef_I);

        F_roca = Matriz_fuerzas_cables(:, 2);
    catch ME
        fprintf('  [!] Trayectoria %d falló en cálculo de fuerzas: %s\n', intentos, ME.message);
        continue;
    end

    %% 5.5 Extraer pares de entrenamiento
    Z_global = Matriz_Trayectoria(:, 3);
    R_global = Matriz_Trayectoria(:, 4);
    L_bit = params_trepano.L_bit;
    idx_valido = Z_global > L_bit & R_global < 10000 & R_global > 0;

    kappa = 1 ./ R_global;
    ROP_col = ones(size(F_roca)) * ROP_mm_min;

    datos_trayectoria = [F_roca(idx_valido), ...
                         kappa(idx_valido), ...
                         ROP_col(idx_valido), ...
                         R_global(idx_valido), ...
                         Z_global(idx_valido)];

    mask = datos_trayectoria(:,1) > 1.0 & datos_trayectoria(:,2) > 0.001;
    datos_trayectoria = datos_trayectoria(mask, :);

    datos_acumulados = [datos_acumulados; datos_trayectoria];

    trayectorias_validas = trayectorias_validas + 1;
    fprintf('  Trayectoria %d/%d: %d puntos | ROP=%.1f mm/min | P1=[%.0f,%.0f,%.0f]\n', ...
        trayectorias_validas, N_trayectorias, size(datos_trayectoria,1), ...
        ROP_mm_min, P1(1),P1(2),P1(3));
end

%% 6. EXPORTAR A CSV
nombre_archivo = 'D:\totig(Usuario)\OneDrive\Documentos\ITBA\Proyecto Final\NN_ROP\dataset_F_k_ROP.csv';

header = {'F_roca_N', 'kappa_1_mm', 'ROP_mm_min', 'R_mm', 'Z_mm'};
fid = fopen(nombre_archivo, 'w');
fprintf(fid, '%s,%s,%s,%s,%s\n', header{:});
fclose(fid);
dlmwrite(nombre_archivo, datos_acumulados, '-append', 'delimiter', ',', 'precision', '%.8f');

%% 7. RESUMEN
fprintf('\n=========================================\n');
fprintf('  RESUMEN DEL DATASET\n');
fprintf('=========================================\n');
fprintf('  Trayectorias generadas: %d\n', trayectorias_validas);
fprintf('  Puntos totales:         %d\n', size(datos_acumulados, 1));
fprintf('  F_roca rango:           [%.2f, %.2f] N\n', min(datos_acumulados(:,1)), max(datos_acumulados(:,1)));
fprintf('  kappa rango:            [%.6f, %.6f] 1/mm\n', min(datos_acumulados(:,2)), max(datos_acumulados(:,2)));
fprintf('  ROP rango:              [%.2f, %.2f] mm/min\n', min(datos_acumulados(:,3)), max(datos_acumulados(:,3)));
fprintf('  Archivo guardado:       %s\n', nombre_archivo);
fprintf('=========================================\n');