%% =========================================================================
% SCRIPT MAIN - TESIS DE GEONAVEGACIÓN
% =========================================================================
clear; clc; close all;

%% 1. Definición de los 3 Waypoints objetivo [X, Y, Z] en mm
% (Z aumenta hacia abajo)
P1 = [0, 0, 120];        
P2 = [64.3, 0, 360];       
P3 = [107.2, 0, 520];       

%% 2. Llamada a la función generadora
% La salida es una matriz Nx6: [X, Y, Z, Radio, Inclinacion, Azimut]
Matriz_Trayectoria = generar_trayectoria(P1, P2, P3);

%% 3. CÁLCULO DE FUERZAS REACTIVAS (MÓDULO DE PLANTA PERNEDER)
% Variables operativas de los actuadores del BHA
RPMhoisting = 3;
ROP_mm_min = (RPMhoisting/2)*(25.4/4);  % Tasa de penetración (ej: 60 mm/min)
RPM = 1500;        % Revoluciones por minuto del motor principal

% Propiedades de la roca (Basado en el Reporte - Saturated Sandstone)
params_roca.epsilon = 25; % Energía específica / UCS [MPa o N/mm^2]
params_roca.zeta_f  = 1;  % Coeficiente de fricción frontal
params_roca.zeta_g  = 10; % Coeficiente de agresividad del calibre

% Geometría del Trépano (Drillbotics Scale)
params_trepano.a = 1.5*25.4/2; % Radio de corte [mm] (Ej: trépano de 40mm de diámetro)
params_trepano.b = 5; % Mitad del largo del calibre [mm]
params_trepano.L_bit = 88;

% Llamada a la función externa
Matriz_Fuerzas = calcular_fuerzas_Perneder(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano);

%% Calculo de la fuerza en los cables
g = 9.81; % [m/s^2] Aceleración de la gravedad

% Brazos de palanca variables del cable (distancia 'l')
l_a = 7.27; % [mm] Brazo de palanca cuando R = R_min_bha 
l_b = 4.11; % [mm] Brazo de palanca cuando R >= R_umbral 

% Brazos de palanca variables del empuje (distancia 'd')
d_a = 10.8;   % [mm] Brazo de palanca cuando R = R_min_bha 
d_b = 13.88;  % [mm] Brazo de palanca cuando R >= R_umbral 

% --- COEFICIENTES POLINOMIO EXCEL (I vs Kg) ---
coef_I = [1.55957502e-04, 3.33106612e-03, 2.37648267e-02, 5.90535677e-02];

% Radios de control
R_min = 186.43; % [mm] Radio de curvatura límite del BHA
R_umbral  = 1000;   % [mm] Radio a partir del cual se considera trayectoria recta

F2 = Matriz_Fuerzas(:,2);
F3 = Matriz_Fuerzas(:,3);

Matriz_fuerzas_cables = calcular_fuerza_actuacion(Matriz_Trayectoria, F2, F3, R_min, R_umbral, l_a, l_b, d_a, d_b, g, coef_I);

P_vector = calcular_fuerza_sarta(Matriz_Trayectoria, 1, 3);

Fuerza_final = Matriz_fuerzas_cables(:,2) + P_vector;
figure; 
plot(Matriz_Trayectoria(:,3), Fuerza_final, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', '#0072BD');
grid on; % Activa la grilla para leer mejor los valores
xlabel('Posición z', 'FontWeight', 'bold');
ylabel('Fuerza [N]', 'FontWeight', 'bold');
title('Fuerza en función de z', 'FontSize', 12);

Fuerza_sarta_mas_pared = calcular_fuerza_final(Matriz_Trayectoria, Fuerza_final, R_min, R_umbral, l_a, l_b, d_a, d_b);

% NUEVA ADICIÓN: DESCOMPOSICIÓN DE FUERZAS EN LOS PADS (PUSH-THE-BIT) 
Matriz_Pads = descomponer_fuerzas_pads(Matriz_Trayectoria, Fuerza_final);

















