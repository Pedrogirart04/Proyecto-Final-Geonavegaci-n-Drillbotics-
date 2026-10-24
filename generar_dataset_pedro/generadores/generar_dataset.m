%% =========================================================================
% GENERAR_DATASET - Genera datos de entrenamiento para la red neuronal
% =========================================================================
% Corre múltiples trayectorias con waypoints aleatorios dentro del cono
% admisible y acumula los pares (F_total, WOB, RPM, kappa) para cada punto.
%
% Salida: archivo CSV con columnas:
%   F_total [N] | kappa [1/mm] | WOB [N] | RPM [rpm] | R [mm] | Z [mm]
%
% Usa las versiones _silent de todas las funciones (sin gráficos).
% =========================================================================
clear; clc; close all;
tic
%% 1. PARÁMETROS DE GENERACIÓN
N_trayectorias = 50;  % <-- Subir a 200 para dataset final
semilla = 42;
rng(semilla);        % Reproducibilidad

%% 2. PARÁMETROS FIJOS DEL SISTEMA (mismos que Main.m)

% Operativos
RPMhoisting = 3;
ROP_mm_min = (RPMhoisting/2) * (25.4/4);
RPM = 1500;

% Roca
params_roca.epsilon = 25;
params_roca.zeta_f  = 1;
params_roca.zeta_g  = 10;

% Trépano
params_trepano.a = 1.5 * 25.4 / 2;
params_trepano.b = 5;
params_trepano.L_bit = 88;

% Sarta
col_deflexion = 1;  % Columna X de Matriz_Trayectoria
col_z = 3;          % Columna Z de Matriz_Trayectoria

% Cables y palanca
g = 9.81;
l_a = 7.27;  l_b = 4.11;
d_a = 10.8;  d_b = 13.88;
coef_I = [1.55957502e-04, 3.33106612e-03, 2.37648267e-02, 5.90535677e-02];
R_min = 186.43;
R_umbral = 1000;

%% 3. LÍMITES DEL CONO ADMISIBLE
% Los waypoints deben respetar:
%   - Desplazamiento horizontal máximo: 100 mm
%   - Inclinación acumulada máxima: 30°
%   - Azimut acumulado máximo: 15°
%   - Z creciente dentro de la muestra (600 mm de profundidad)
%   - Trayectoria tipo S

% Rangos de Z para cada waypoint
Z1_range = [120, 220];
Z2_range = [260, 400];
Z3_range = [450, 580];

% Desplazamiento horizontal máximo
XY_max = 100;  % [mm]

%% 4. ACUMULADOR DE DATOS
% Columnas: F_total | kappa | WOB | RPM | R | Z
datos_acumulados = [];

%% 5. LOOP DE GENERACIÓN
fprintf('=========================================\n');
fprintf('  GENERACIÓN DE DATASET\n');
fprintf('=========================================\n');

trayectorias_validas = 0;
intentos = 0;
max_intentos = N_trayectorias * 5;  % Margen para trayectorias inválidas

while trayectorias_validas < N_trayectorias && intentos < max_intentos
    intentos = intentos + 1;
    
    %% 5.1 Generar waypoints aleatorios dentro del cono
    Z1 = Z1_range(1) + rand() * (Z1_range(2) - Z1_range(1));
    Z2 = Z2_range(1) + rand() * (Z2_range(2) - Z2_range(1));
    Z3 = Z3_range(1) + rand() * (Z3_range(2) - Z3_range(1));
    
    epsilon_base = 22 + rand() * 6;  % Uniforme en [22, 28]
    params_roca.epsilon = epsilon_base;

    % Desplazamiento horizontal progresivo (crece con Z)
    % Para que sea tipo S, el desplazamiento crece y luego puede cambiar dirección
    X1 = (rand() * 0.3) * XY_max;           % Pequeño al principio
    X2 = (0.3 + rand() * 0.4) * XY_max;     % Medio
    X3 = (0.5 + rand() * 0.5) * XY_max;     % Mayor al final
    
    Y1 = (rand() - 0.5) * 0.3 * XY_max;     % Pequeña variación en Y
    Y2 = (rand() - 0.5) * 0.5 * XY_max;
    Y3 = (rand() - 0.5) * 0.6 * XY_max;
    
    % Verificar desplazamiento horizontal total
    desp_horiz = sqrt(X3^2 + Y3^2);
    if desp_horiz > XY_max
        continue;  % Descartar este set de waypoints
    end
    
    P1 = [X1, Y1, Z1];
    P2 = [X2, Y2, Z2];
    P3 = [X3, Y3, Z3];
    
    %% 5.2 Generar trayectoria
    try
        Matriz_Trayectoria = generar_trayectoria_silent(P1, P2, P3);
    catch
        fprintf('  [!] Trayectoria %d falló en generación, descartada.\n', intentos);
        continue;
    end
    
    % Verificar radio mínimo
    R_tray = Matriz_Trayectoria(:, 4);
    R_min_real = min(R_tray(R_tray > 0 & R_tray < 10000));
    if isempty(R_min_real) || R_min_real < R_min
        fprintf('  [!] Trayectoria %d viola R_min (R=%.1f mm), descartada.\n', intentos, R_min_real);
        continue;
    end
    
    %% 5.3 Calcular fuerzas (cadena completa)
    try
        % Fuerzas de la roca (Perneder)
        Matriz_Fuerzas = calcular_fuerzas_Perneder_silent(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano);
        
        F2 = Matriz_Fuerzas(:, 2);
        F3 = Matriz_Fuerzas(:, 3);
        
        % Fuerzas en cables
        Matriz_fuerzas_cables = calcular_fuerza_actuacion_silent(Matriz_Trayectoria, F2, F3, R_min, R_umbral, l_a, l_b, d_a, d_b, g, coef_I);
        
        % Fuerza restitutiva de la sarta
        P_vector = calcular_fuerza_sarta_silent(Matriz_Trayectoria, col_deflexion, col_z);
        
        % Fuerza total
        Fuerza_final = Matriz_fuerzas_cables(:, 2) + P_vector;
        
        % Fuerza final con palanca
        Fuerza_sarta_mas_pared = calcular_fuerza_final_silent(Matriz_Trayectoria, Fuerza_final, R_min, R_umbral, l_a, l_b, d_a, d_b);
        
        % Descomposición en pads
        Matriz_Pads = descomponer_fuerzas_pads_silent(Matriz_Trayectoria, Fuerza_final);
        
    catch ME
        fprintf('  [!] Trayectoria %d falló en cálculo de fuerzas: %s\n', intentos, ME.message);
        continue;
    end
    
    %% 5.4 Extraer pares de entrenamiento
    Z_global = Matriz_Trayectoria(:, 3);
    R_global = Matriz_Trayectoria(:, 4);
    
    % Solo puntos del tramo curvo (después de L_bit)
    L_bit = params_trepano.L_bit;
    idx_valido = Z_global > L_bit & R_global < 10000 & R_global > 0;
    
    % Curvatura
    kappa = 1 ./ R_global;
    
    % F_total es la magnitud de fuerza total que los pads ejercen
    F_roca = Matriz_fuerzas_cables(:,2);
    
    % WOB y RPM son constantes en esta simulación
    WOB_col = ones(size(F_roca)) * (ROP_mm_min * params_roca.epsilon * params_trepano.a * pi/2);
    RPM_col = ones(size(F_roca)) * RPM;
    
   
    epsilon_col = ones(size(F_roca)) * epsilon_base;
    % Filtrar puntos válidos
    datos_trayectoria = [F_roca(idx_valido), ...
                     kappa(idx_valido), ...
                     epsilon_col(idx_valido), ...
                     WOB_col(idx_valido), ...
                     RPM_col(idx_valido), ...
                     R_global(idx_valido), ...
                     Z_global(idx_valido)];
    
    % Descartar puntos con F_total <= 0 o kappa <= 0
    mask = datos_trayectoria(:,1) > 1.0 & datos_trayectoria(:,2) > 0;
    datos_trayectoria = datos_trayectoria(mask, :);
    
    datos_acumulados = [datos_acumulados; datos_trayectoria];
    
    trayectorias_validas = trayectorias_validas + 1;
    fprintf('  Trayectoria %d/%d: %d puntos válidos | P1=[%.0f,%.0f,%.0f] P2=[%.0f,%.0f,%.0f] P3=[%.0f,%.0f,%.0f]\n', ...
        trayectorias_validas, N_trayectorias, size(datos_trayectoria,1), ...
        P1(1),P1(2),P1(3), P2(1),P2(2),P2(3), P3(1),P3(2),P3(3));
end

%% 6. EXPORTAR A CSV
nombre_archivo = 'D:\totig(Usuario)\OneDrive\Documentos\ITBA\Proyecto Final\NN_1\dataset_F_k_UCS.csv';

% Header
header = {'F_roca_N', 'kappa_1_mm', 'epsilon_MPa', 'WOB_N', 'RPM', 'R_mm', 'Z_mm'};

% Escribir CSV
fid = fopen(nombre_archivo, 'w');
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s\n', header{:});
fclose(fid);
dlmwrite(nombre_archivo, datos_acumulados, '-append', 'delimiter', ',', 'precision', '%.8f');

%% 7. RESUMEN
fprintf('\n=========================================\n');
fprintf('  RESUMEN DEL DATASET\n');
fprintf('=========================================\n');
fprintf('  Trayectorias generadas: %d\n', trayectorias_validas);
fprintf('  Puntos totales:         %d\n', size(datos_acumulados, 1));
fprintf('  F_total rango:          [%.2f, %.2f] N\n', min(datos_acumulados(:,1)), max(datos_acumulados(:,1)));
fprintf('  kappa rango:            [%.6f, %.6f] 1/mm\n', min(datos_acumulados(:,2)), max(datos_acumulados(:,2)));
fprintf('  Archivo guardado:       %s\n', nombre_archivo);
fprintf('=========================================\n');

toc