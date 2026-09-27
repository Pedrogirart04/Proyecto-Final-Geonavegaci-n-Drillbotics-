%% =========================================================================
% GENERADOR DE TRAYECTORIA DIRECCIONAL - PROYECTO DRILLBOTICS
% Autor: Conrado Besel Stur & Equipo
% =========================================================================
clear; clc; close all;

%% 1. PARÁMETROS FÍSICOS Y WAYPOINTS
L_bit = 90;     % Largo de la sección rígida inicial [mm]
R_min = 196.65;      % Radio de curvatura mínimo admisible [mm]
K_max = 1/R_min; % Curvatura máxima permitida [1/mm]

% Coordenadas de los puntos a perseguir [X, Y, Z] en mm.
% Z=0 es la superficie, Z aumenta hacia abajo (profundidad).
P0 = [0, 0, 0];          % Superficie
P1 = [0, 0, L_bit];      % Fin del tramo rígidamente vertical
P2 = [0, 0, 200];        
P3 = [40, 5, 400];       
P4 = [50, 10, 600];       
P_global = [P0; P1; P2; P3; P4]'; 

%% 2. CONSTRUCCIÓN TRAMO 1: ENTRADA ESTRICTAMENTE RECTA
N_recto = 200; 
Z_recto = linspace(0, L_bit, N_recto);
X_recto = zeros(1, N_recto);
Y_recto = zeros(1, N_recto);

%% 3. CONSTRUCCIÓN TRAMO 2: CURVA DIRECCIONAL (PCHIP)
% Aislamos los puntos que corresponden a la curva
P_curva = [P1; P2; P3; P4]'; 

% Parametrización por Distancia Acumulada (Chordal Length)
% Esto mejora drásticamente cómo fluye la curva entre los puntos
dists = sqrt(sum(diff(P_curva, 1, 2).^2, 1));
t_curva = [0, cumsum(dists)];
t_curva = t_curva / t_curva(end); % Normalizamos de 0 a 1

N_curva_fino = 1000;
t_fino_curva = linspace(0, 1, N_curva_fino);

% --- LA MAGIA CONTRA EL SALTO: PCHIP ---
% Al usar 'pchip', forzamos a que el tramo entre P1 y P2 sea una línea 
% recta perfecta sin overshoots hacia atrás.
Trayectoria_Curva = zeros(3, N_curva_fino);
Trayectoria_Curva(1,:) = pchip(t_curva, P_curva(1,:), t_fino_curva);
Trayectoria_Curva(2,:) = pchip(t_curva, P_curva(2,:), t_fino_curva);
Trayectoria_Curva(3,:) = pchip(t_curva, P_curva(3,:), t_fino_curva);

%% 4. ENSAMBLE TOTAL DE LA TRAYECTORIA
% Unimos el tramo recto con la curva (descartamos el primer punto de la curva)
X = [X_recto, Trayectoria_Curva(1, 2:end)];
Y = [Y_recto, Trayectoria_Curva(2, 2:end)];
Z = [Z_recto, Trayectoria_Curva(3, 2:end)];

%% 5. ANÁLISIS GEOMÉTRICO Y CURVATURA
dX_c = gradient(Trayectoria_Curva(1, :), t_fino_curva);
dY_c = gradient(Trayectoria_Curva(2, :), t_fino_curva);
dZ_c = gradient(Trayectoria_Curva(3, :), t_fino_curva);
ddX_c = gradient(dX_c, t_fino_curva);
ddY_c = gradient(dY_c, t_fino_curva);
ddZ_c = gradient(dZ_c, t_fino_curva);
ds_c = sqrt(dX_c.^2 + dY_c.^2 + dZ_c.^2);

% Curvatura en el espacio 3D 
Numerador_K = sqrt((dY_c.*ddZ_c - dZ_c.*ddY_c).^2 + ...
                   (dZ_c.*ddX_c - dX_c.*ddZ_c).^2 + ...
                   (dX_c.*ddY_c - dY_c.*ddX_c).^2);
Curvatura_Curva = Numerador_K ./ (ds_c.^3);

% Curvatura total de la trayectoria
Curvatura = [zeros(1, N_recto), Curvatura_Curva(2:end)];
Max_Curvatura = max(Curvatura);
Min_Radio_Real = 1 / Max_Curvatura;

%% 6. EXPORTACIÓN DE VARIABLES PARA EL CONTROLADOR (FEEDFORWARD)
dX_tot = diff(X);
dY_tot = diff(Y);
dZ_tot = diff(Z);
ds_tot = sqrt(dX_tot.^2 + dY_tot.^2 + dZ_tot.^2);

S_perforada = [0, cumsum(ds_tot)]; 

Theta_ref = acos(dZ_tot ./ ds_tot);
Theta_ref = [Theta_ref, Theta_ref(end)]; 
Phi_ref = atan2(dY_tot, dX_tot);
Phi_ref = unwrap(Phi_ref);               
Phi_ref = [Phi_ref, Phi_ref(end)];       

% Convertimos a grados 
Theta_deg = Theta_ref * (180/pi);
Phi_deg = Phi_ref * (180/pi);

% --- FILTRO DE SINGULARIDAD VERTICAL ---
% Si la inclinación es menor a 0.5 grados, forzamos el Azimut a 0 
% para evitar el ruido numérico del "Gimbal Lock".
umbral_vertical = 0.5; % Grados
Phi_deg(Theta_deg < umbral_vertical) = 0;
Phi_ref = Phi_deg * (pi/180); % Actualizamos también en radianes por las dudas

% --- REPORTE EN CONSOLA ---
fprintf('==================================================\n');
fprintf('   REPORTE DE TRAYECTORIA DIRECCIONAL\n');
fprintf('==================================================\n');
fprintf(' Longitud total a perforar (S): %.2f mm\n', S_perforada(end));
fprintf(' Radio de curvatura mínimo logrado: %.2f mm\n', Min_Radio_Real);
fprintf('--------------------------------------------------\n');
if Min_Radio_Real < R_min
    fprintf(2, ' [!] ALERTA CRÍTICA: Radio menor a %.2f mm.\n', R_min);
    fprintf(2, '     El equipo se atascará. Ajustar waypoints.\n');
else
    disp(' [OK] Geometría fluida y dentro de los límites físicos.');
end
fprintf('==================================================\n');

%% 7. VISUALIZACIÓN 3D
fig1 = figure('Name', 'Generador de Trayectoria 3D', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.05 0.1 0.4 0.8]);

% Adaptación para mostrar Radio de Curvatura directo
Radio_Curvatura = 1 ./ (Curvatura + 1e-12);
Radio_Max_Visual = 1500; 
Radio_Curvatura(Radio_Curvatura > Radio_Max_Visual) = Radio_Max_Visual;

scatter3(X, Y, Z, 15, Radio_Curvatura, 'filled'); 
hold on; grid on; axis equal; 

plot3(P_global(1,:), P_global(2,:), P_global(3,:), 'ko', 'MarkerSize', 7, 'LineWidth', 2, 'MarkerFaceColor', 'w'); 

% Invertimos el mapa para que lo peligroso (radio bajo) sea rojo
colormap(flipud(jet));
cb = colorbar;
ylabel(cb, 'Radio de Curvatura R [mm]', 'FontSize', 10);
caxis([R_min Radio_Max_Visual]); 

title('Trayectoria Planificada (3D)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Desplazamiento Eje X [mm]');
ylabel('Desplazamiento Eje Y [mm]');
zlabel('Profundidad Eje Z [mm]');
set(gca, 'ZDir', 'reverse'); 
legend('Trayectoria', 'Waypoints', 'Location', 'best');
view(35, 25);

%% 8. PLANOS DE TRAYECTORIA (2D)
fig2 = figure('Name', 'Planos Geométricos 2D', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.45 0.5 0.5 0.45]);

subplot(1, 3, 1);
plot(X, Y, 'b-', 'LineWidth', 2); hold on; grid on;
plot(P_global(1,:), P_global(2,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
title('Plano XY (Vista Superior)'); xlabel('X [mm]'); ylabel('Y [mm]');
axis equal;

subplot(1, 3, 2);
plot(X, Z, 'r-', 'LineWidth', 2); hold on; grid on;
plot(P_global(1,:), P_global(3,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
title('Plano XZ (Vista Frontal)'); xlabel('X [mm]'); ylabel('Profundidad Z [mm]');
set(gca, 'YDir', 'reverse'); 
axis equal;

subplot(1, 3, 3);
plot(Y, Z, 'g-', 'LineWidth', 2); hold on; grid on;
plot(P_global(2,:), P_global(3,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
title('Plano YZ (Vista Lateral)'); xlabel('Y [mm]'); ylabel('Profundidad Z [mm]');
set(gca, 'YDir', 'reverse'); 
axis equal;

%% 9. EVOLUCIÓN DE ÁNGULOS DE CONTROL
fig3 = figure('Name', 'Variables de Control', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.45 0.05 0.5 0.45]);

subplot(1, 2, 1);
plot(Z, Theta_deg, 'k-', 'LineWidth', 2); grid on; hold on;
xline(L_bit, 'r--', 'Fin Tramo Rígido', 'LabelHorizontalAlignment', 'left');
title('Inclinación (\Theta) vs Profundidad');
xlabel('Profundidad Z [mm]'); ylabel('Inclinación [Grados]');
xlim([0 max(Z)]);

subplot(1, 2, 2);
plot(Z, Phi_deg, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2); grid on; hold on;
xline(L_bit, 'r--', 'Fin Tramo Rígido', 'LabelHorizontalAlignment', 'left');
title('Azimut (\Phi) vs Profundidad');
xlabel('Profundidad Z [mm]'); ylabel('Azimut [Grados]');
xlim([0 max(Z)]);