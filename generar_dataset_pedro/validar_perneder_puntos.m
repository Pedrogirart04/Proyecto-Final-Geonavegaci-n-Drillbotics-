%% =========================================================================
% VALIDAR_PERNEDER_PUNTOS - Validación 2: F_roca analítico en puntos elegidos
% =========================================================================
% Calcula F_roca con la fórmula analítica de Perneder (la misma que generó
% el dataset de entrenamiento) para un conjunto de puntos (kappa, ROP, RPM,
% UCS) elegidos a propósito, y los guarda en CSV para comparar en Python
% contra lo que predice la red entrenada.
% =========================================================================
clear; clc; close all;

%% 1. PUNTOS A EVALUAR (elegidos a propósito, no necesariamente del dataset)
% Formato: cada fila es [kappa_1_mm, ROP_mm_min, RPM, UCS_MPa]
% 
puntos = [
    0.0010, 10, 1200, 22;
    0.0020, 10, 1200, 22;
    0.0030, 10, 1200, 22;
    0.0040, 15, 1500, 25;
    0.0050, 15, 1500, 25;
    0.0027, 20, 1800, 28;
    0.0027,  5, 1000, 20;
    0.0027, 30, 2000, 30;
];

N = size(puntos, 1);
kappa_vals = puntos(:,1);
ROP_vals   = puntos(:,2);
RPM_vals   = puntos(:,3);
UCS_vals   = puntos(:,4);
R_vals     = 1 ./ kappa_vals;   % Perneder trabaja con radio, no curvatura

%% 2. PARÁMETROS FÍSICOS FIJOS (iguales a generar_dataset_v2.m)
params_roca.zeta_f = 1;
params_roca.zeta_g = 10;
params_trepano.a = 1.5 * 25.4 / 2;
params_trepano.b = 5;
params_trepano.L_bit = 88;

%% 3. CALCULAR F_roca ANALÍTICO, UN PUNTO A LA VEZ
% (ROP/RPM/UCS entran como escalares por llamada, así que hay que llamar
% a la función una vez por punto, no se puede vectorizar con valores
% distintos de ROP/RPM/UCS en una sola llamada)
F_roca_analitico = zeros(N, 1);
for i = 1:N
    params_roca.epsilon = UCS_vals(i);

    % Dos filas: la real + una de relleno. La función internamente calcula
    % un marco de Frenet-Serret (que no usamos) con diff() entre filas, y
    % eso se rompe con una sola fila -- la fila de relleno es solo para
    % que no falle, no afecta el resultado de la fila 1 que sí leemos.
    R_par = [R_vals(i); R_vals(i)];
    Z_par = [1; 2];
    Matriz_Trayectoria_i = [zeros(2,1), zeros(2,1), Z_par, R_par];

    Matriz_Fuerzas = calcular_fuerzas_Perneder_silent(Matriz_Trayectoria_i, ROP_vals(i), RPM_vals(i), params_roca, params_trepano);
    F2 = Matriz_Fuerzas(1, 2);
    F3 = Matriz_Fuerzas(1, 3);
    F_roca_analitico(i) = sqrt(F2^2 + F3^2);
end

%% 4. EXPORTAR
T = table(kappa_vals, ROP_vals, RPM_vals, UCS_vals, F_roca_analitico);
disp(T);
writetable(T, 'validacion_perneder_puntos.csv');