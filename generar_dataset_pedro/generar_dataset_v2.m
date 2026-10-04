%% =========================================================================
% GENERAR_DATASET_V2 - Dataset de entrenamiento para la red direccional
%                      (ROP, RPM, UCS, kappa) -> F_roca
% =========================================================================
% Reemplaza a generar_dataset.m y generador_dataset_F_k_ROP.m: en vez de
% fijar 2 de las 3 variables operativas y barrer solo una, acá se sortean
% ROP, RPM y UCS(epsilon) juntas por trayectoria usando Latin Hypercube
% Sampling (LHS), para cubrir el cubo 3D de forma pareja con pocas corridas.
% kappa no se sortea: sale gratis de los muchos puntos de cada trayectoria.
%
% Salida CSV: F_roca_N | kappa_1_mm | ROP_mm_min | RPM | epsilon_MPa |
%             R_mm | Z_mm | trayectoria_id
%
% Usa generar_trayectoria_v2_silent.m (tramo recto vertical real hasta
% Z_INICIO_CURVA, ver tarea 1 de Fase B).
% =========================================================================
clear; clc; close all;
tic

%% 1. PARÁMETROS DE GENERACIÓN
N_trayectorias = 200;   % <<<< AJUSTAR SI CAMBIA: cantidad final de trayectorias
semilla = 42;
rng(semilla);           % Reproducibilidad (afecta waypoints Y el LHS)

%% 2. RANGOS DE LAS VARIABLES OPERATIVAS (sorteadas via LHS)
ROP_range = [5, 20];       % [mm/min]  <<<< AJUSTAR SI CAMBIA
RPM_range = [1000, 2000];  % [rpm]     <<<< AJUSTAR SI CAMBIA (valor provisorio, a confirmar con el equipo)
UCS_range = [22, 28];      % [MPa]     <<<< AJUSTAR SI CAMBIA

%% 3. PARÁMETROS FIJOS DEL SISTEMA (mismos que Main.m / generar_dataset.m)

% Roca (zeta fijos, epsilon se sortea mas abajo)
params_roca.zeta_f  = 1;
params_roca.zeta_g  = 10;

% Trépano
params_trepano.a = 1.5 * 25.4 / 2;
params_trepano.b = 5;
params_trepano.L_bit = 88;   % Largo físico real del BHA [mm]. NO se usa en
                              % el cálculo de fuerzas (ver calcular_fuerzas_
                              % Perneder_silent.m línea 21, queda sin usar),
                              % se mantiene solo porque la función espera el
                              % campo. No confundir con Z_INICIO_CURVA abajo.

% Geometría de la trayectoria: profundidad del primer waypoint (fin del
% tramo recto / inicio de la curva). Debe coincidir con el default de
% generar_trayectoria_v2_silent.m.
Z_INICIO_CURVA = 120;   % [mm] <<<< AJUSTAR SI CAMBIA (y también en generar_trayectoria_v2_silent.m)

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

%% 4. LÍMITES DEL CONO ADMISIBLE (waypoints)
% Solo 2 waypoints random: P1 queda fijo en [0,0,Z_INICIO_CURVA] (sección 3)
Z_P2_range = [260, 400];
Z_P3_range = [450, 580];
XY_max = 100;  % [mm]

%% 5. LATIN HYPERCUBE SAMPLING DE (ROP, RPM, UCS)
% Una fila por trayectoria FINALMENTE válida (no por intento). Si una
% combinación falla por geometría, se reintenta con la misma fila y
% nuevos waypoints, así la estratificación LHS se preserva en el dataset final.
lhs_samples = lhs_manual(N_trayectorias, 3);
ROP_vals = ROP_range(1) + lhs_samples(:,1) * diff(ROP_range);
RPM_vals = RPM_range(1) + lhs_samples(:,2) * diff(RPM_range);
UCS_vals = UCS_range(1) + lhs_samples(:,3) * diff(UCS_range);

%% 6. ACUMULADOR DE DATOS
datos_acumulados = [];

%% 7. LOOP DE GENERACIÓN
fprintf('=========================================\n');
fprintf('  GENERACIÓN DE DATASET V2 (ROP, RPM, UCS)\n');
fprintf('=========================================\n');

trayectorias_validas = 0;
intentos = 0;
max_intentos = N_trayectorias * 5;

while trayectorias_validas < N_trayectorias && intentos < max_intentos
    intentos = intentos + 1;
    idx_lhs = trayectorias_validas + 1;

    %% 7.1 Tomar la terna (ROP, RPM, UCS) de esta posición del LHS
    ROP_mm_min = ROP_vals(idx_lhs);
    RPM = RPM_vals(idx_lhs);
    params_roca.epsilon = UCS_vals(idx_lhs);

    %% 7.2 Generar waypoints aleatorios dentro del cono (P1 fijo, no se sortea)
    Z_P2 = Z_P2_range(1) + rand() * (Z_P2_range(2) - Z_P2_range(1));
    Z_P3 = Z_P3_range(1) + rand() * (Z_P3_range(2) - Z_P3_range(1));

    X_P2 = (0.3 + rand() * 0.4) * XY_max;   % desplazamiento medio
    X_P3 = (0.5 + rand() * 0.5) * XY_max;   % desplazamiento mayor al final

    Y_P2 = (rand() - 0.5) * 0.5 * XY_max;
    Y_P3 = (rand() - 0.5) * 0.6 * XY_max;

    desp_horiz = sqrt(X_P3^2 + Y_P3^2);
    if desp_horiz > XY_max
        continue;
    end

    P2 = [X_P2, Y_P2, Z_P2];
    P3 = [X_P3, Y_P3, Z_P3];

    %% 7.3 Generar trayectoria (P1 fijo en [0,0,Z_INICIO_CURVA], ver v2)
    try
        Matriz_Trayectoria = generar_trayectoria_v2_silent(P2, P3, Z_INICIO_CURVA);
    catch
        fprintf('  [!] Intento %d falló en generación, descartado.\n', intentos);
        continue;
    end

    R_tray = Matriz_Trayectoria(:, 4);
    R_min_real = min(R_tray(R_tray > 0 & R_tray < 10000));
    if isempty(R_min_real) || R_min_real < R_min
        fprintf('  [!] Intento %d viola R_min (R=%.1f mm), descartado.\n', intentos, R_min_real);
        continue;
    end
    %% 7.4 Calcular fuerzas (cadena completa)
    try
        Matriz_Fuerzas = calcular_fuerzas_Perneder_silent(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano);
        F2 = Matriz_Fuerzas(:, 2);
        F3 = Matriz_Fuerzas(:, 3);

        Matriz_fuerzas_cables = calcular_fuerza_actuacion_silent(Matriz_Trayectoria, F2, F3, R_min, R_umbral, l_a, l_b, d_a, d_b, g, coef_I);
    catch ME
        fprintf('  [!] Intento %d falló en cálculo de fuerzas: %s\n', intentos, ME.message);
        continue;
    end

    %% 7.5 Extraer pares de entrenamiento
    Z_global = Matriz_Trayectoria(:, 3);
    R_global = Matriz_Trayectoria(:, 4);

    % Solo puntos del tramo curvo (después de Z_INICIO_CURVA, NO de L_bit)
    idx_valido = Z_global > Z_INICIO_CURVA & R_global < 10000 & R_global > 0;

    kappa = 1 ./ R_global;
    F_roca = Matriz_fuerzas_cables(:, 2);

    ROP_col = ones(size(F_roca)) * ROP_mm_min;
    RPM_col = ones(size(F_roca)) * RPM;
    UCS_col = ones(size(F_roca)) * params_roca.epsilon;
    trayectoria_id_col = ones(size(F_roca)) * idx_lhs;

    datos_trayectoria = [F_roca(idx_valido), ...
                         kappa(idx_valido), ...
                         ROP_col(idx_valido), ...
                         RPM_col(idx_valido), ...
                         UCS_col(idx_valido), ...
                         R_global(idx_valido), ...
                         Z_global(idx_valido), ...
                         trayectoria_id_col(idx_valido)];

    mask = datos_trayectoria(:,1) > 1.0 & datos_trayectoria(:,2) > 0;
    datos_trayectoria = datos_trayectoria(mask, :);

    datos_acumulados = [datos_acumulados; datos_trayectoria];

    trayectorias_validas = trayectorias_validas + 1;
    fprintf('  Trayectoria %d/%d: %d puntos | ROP=%.1f RPM=%.0f UCS=%.1f | P2=[%.0f,%.0f,%.0f]\n', ...
        trayectorias_validas, N_trayectorias, size(datos_trayectoria,1), ...
        ROP_mm_min, RPM, params_roca.epsilon, P2(1),P2(2),P2(3));
end

%% 8. EXPORTAR A CSV
nombre_archivo = 'dataset_F_k_ROP_RPM_UCS.csv';   % <<<< AJUSTAR RUTA/NOMBRE SI HACE FALTA

header = {'F_roca_N', 'kappa_1_mm', 'ROP_mm_min', 'RPM', 'epsilon_MPa', 'R_mm', 'Z_mm', 'trayectoria_id'};
fid = fopen(nombre_archivo, 'w');
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s,%s\n', header{:});
fclose(fid);
dlmwrite(nombre_archivo, datos_acumulados, '-append', 'delimiter', ',', 'precision', '%.8f');

%% 9. RESUMEN
fprintf('\n=========================================\n');
fprintf('  RESUMEN DEL DATASET\n');
fprintf('=========================================\n');
fprintf('  Trayectorias generadas: %d\n', trayectorias_validas);
fprintf('  Puntos totales:         %d\n', size(datos_acumulados, 1));
fprintf('  F_roca rango:           [%.2f, %.2f] N\n', min(datos_acumulados(:,1)), max(datos_acumulados(:,1)));
fprintf('  kappa rango:            [%.6f, %.6f] 1/mm\n', min(datos_acumulados(:,2)), max(datos_acumulados(:,2)));
fprintf('  ROP rango:              [%.2f, %.2f] mm/min\n', min(datos_acumulados(:,3)), max(datos_acumulados(:,3)));
fprintf('  RPM rango:              [%.1f, %.1f]\n', min(datos_acumulados(:,4)), max(datos_acumulados(:,4)));
fprintf('  UCS rango:              [%.2f, %.2f] MPa\n', min(datos_acumulados(:,5)), max(datos_acumulados(:,5)));
fprintf('  Archivo guardado:       %s\n', nombre_archivo);
fprintf('=========================================\n');

toc