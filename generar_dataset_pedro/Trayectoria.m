%% =========================================================================
% GENERADOR DE TRAYECTORIA CONTÍNUA CON FILTRO DE SEÑAL PARA SIMULINK
% Método de Azimut por Diferencias Centrales (Vectores Secantes)
% Autor: Conrado Besel Stur & Equipo - Proyecto Drillbotics ITBA
% =========================================================================
clear; clc; close all;

%% 1. PARÁMETROS FÍSICOS Y WAYPOINTS
L_bit = 88;     % Largo de la sección rígida inicial [mm]
R_min = 196.65; % Radio de curvatura mínimo admisible [mm]

% Coordenadas de los puntos a perseguir [X, Y, Z] en mm.
P0 = [0, 0, 0];          
P1 = [0, 0, L_bit];      
P2 = [0, 0, 200];        
P3 = [40, 5, 400];       
P4 = [50, 10, 600];       
P_global = [P0; P1; P2; P3; P4]'; 

%% 2. PARAMETRIZACIÓN POR DISTANCIA ACUMULADA
dists = sqrt(sum(diff(P_global, 1, 2).^2, 1));
t_waypoints = [0, cumsum(dists)];
N_puntos = 1200;
t_fino = linspace(0, t_waypoints(end), N_puntos);

%% 3. INTERPOLACIÓN POR SPLINE SUJETO
v_start = [0; 0; 1];
v_end = (P_global(:,end) - P_global(:,end-1)) / dists(end);

trayectoria = zeros(3, N_puntos);
for coord = 1:3
    y_interp = [v_start(coord), P_global(coord,:), v_end(coord)];
    trayectoria(coord, :) = spline(t_waypoints, y_interp, t_fino);
end

X = trayectoria(1, :); Y = trayectoria(2, :); Z = trayectoria(3, :);

%% 4. VARIABLES DE CONTROL: INCLINACIÓN (THETA)
dX = gradient(X, t_fino); dY = gradient(Y, t_fino); dZ = gradient(Z, t_fino);
ds = sqrt(dX.^2 + dY.^2 + dZ.^2);
S_perforada = [0, cumsum(ds(1:end-1) .* diff(t_fino))];
Theta_ref = acos(dZ ./ ds);
Theta_deg = Theta_ref * (180/pi);

%% 5. CÁLCULO DE AZIMUT POR VECTORES SECANTES (MÉTODO CONRA + FIX BLINDADO)
k = 10; % Usamos solo la ventana de caño rígido
Phi_raw = zeros(1, N_puntos);

% Calculamos el Azimut base usando vectores secantes
for i = 1:N_puntos
    idx_atras = max(1, i-k);
    idx_adelante = min(N_puntos, i+k);
    vec_X = X(idx_adelante) - X(idx_atras);
    vec_Y = Y(idx_adelante) - Y(idx_atras);
    Phi_raw(i) = atan2(vec_Y, vec_X);
end

% --- ELIMINACIÓN ABSOLUTA DE LA MONTAÑA (SPLINE OVERSHOOT) ---
% Ángulo geométrico exacto hacia el 1er waypoint (P3 respecto a P2)
Angulo_Salida_Ideal = atan2(P_global(2,4) - P_global(2,3), P_global(1,4) - P_global(1,3)); 

% 1. Encontramos la profundidad donde empezamos a orientar la herramienta (Z=100)
idx_z100 = find(Z >= 100, 1, 'first'); 

% 2. Buscamos el momento exacto donde la matemática del spline se rinde,
% deja de hacer "panza" hacia atrás, y finalmente apunta hacia nuestro objetivo.
% Le pedimos que esté cerca del ángulo ideal (< 0.05 rad) y que ya esté en la curva (Z>200).
idx_confiable = find(Z > 200 & abs(Phi_raw - Angulo_Salida_Ideal) < 0.05, 1, 'first');

if isempty(idx_confiable)
    idx_confiable = find(Z >= 250, 1, 'first'); % Seguro de vida por si las dudas
end

% APLICAMOS LA FUERZA BRUTA LÓGICA:
% Arranque clavado en CERO absoluto hasta los 100 mm.
Phi_raw(1:idx_z100) = 0;
% Toolface apuntado perfecto hasta que el spline sea confiable.
Phi_raw(idx_z100+1:idx_confiable) = Angulo_Salida_Ideal;

Phi_raw = unwrap(Phi_raw);

% --- EL FILTRO DEFINITIVO (DOBLE PASADA) ---
% Filtro mediano para asegurar que no quede ningún pico residual
Phi_sin_montana = smoothdata(Phi_raw, 'movmedian', 100); 
% Filtro gaussiano fuerte para convertir el escalón de 0 a 7° en una rampa perfecta
Phi_oficial_rad = smoothdata(Phi_sin_montana, 'gaussian', 100); 

% 1. AZIMUT ABSOLUTO: Brújula clásica normalizada de 0 a 360 grados.
Phi_deg_absoluto = mod(Phi_oficial_rad * (180/pi), 360);

% 2. AZIMUT ACUMULADO (Lógica Conra): Sumamos cuánto giró sin importar el signo.
delta_phi_deg = diff(Phi_oficial_rad * (180/pi));
Phi_deg_acumulado = [0, cumsum(abs(delta_phi_deg))];

%% 6. ANÁLISIS DE CURVATURA ANALÍTICA 3D 
ddX = gradient(dX, t_fino); ddY = gradient(dY, t_fino); ddZ = gradient(dZ, t_fino);
Numerador_K = sqrt((dY.*ddZ - dZ.*ddY).^2 + (dZ.*ddX - dX.*ddZ).^2 + (dX.*ddY - dY.*ddX).^2);
Curvatura = Numerador_K ./ (ds.^3);
Max_Curvatura = max(Curvatura(t_fino > L_bit)); 
Min_Radio_Real = 1 / (Max_Curvatura + 1e-12);

%% 7. REPORTE EN CONSOLA
fprintf('==================================================\n');
fprintf('     REPORTE DE TRAYECTORIA (ARRANQUE EN 0°)      \n');
fprintf('==================================================\n');
fprintf(' Longitud total del pozo (S):     %.2f mm\n', S_perforada(end));
fprintf(' Radio de curvatura mínimo real:  %.2f mm\n', Min_Radio_Real);
fprintf(' Esfuerzo de rotación total:      %.2f°\n', Phi_deg_acumulado(end));
fprintf('--------------------------------------------------\n');
if Min_Radio_Real < R_min
    fprintf(2, ' [!] ALERTA CRÍTICA: El radio infringe el límite de %0.2f mm.\n', R_min);
else
    disp(' [OK] Radio seguro. Señales de control suavizadas.');
end
fprintf('==================================================\n');

%% 8. GRÁFICOS DE VARIABLES DE CONTROL BÁSICAS
fig1 = figure('Name', 'Variables de Control', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.8 0.45]);
subplot(1, 2, 1);
plot(Z, Theta_deg, 'k-', 'LineWidth', 2.5); grid on; hold on;
xline(L_bit, 'r--', 'Fin Tramo Rígido (88 mm)', 'LabelHorizontalAlignment', 'left');
title('Inclinación (\Theta) vs Profundidad');
xlabel('Profundidad Z [mm]'); ylabel('Inclinación [Grados]');
xlim([0 max(Z)]); ylim([0 max(Theta_deg)+2]);

subplot(1, 2, 2);
plot(Z, Phi_deg_absoluto, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5); hold on; grid on;
xline(L_bit, 'r--', 'Fin Tramo Rígido (88 mm)', 'LabelHorizontalAlignment', 'left');
title('Azimut (\Phi) Oficial vs Profundidad');
xlabel('Profundidad Z [mm]'); ylabel('Azimut Absoluto [Grados]');
xlim([0 max(Z)]);

%% 9. VISUALIZACIÓN TRAYECTORIA 3D
fig2 = figure('Name', 'Trayectoria 3D liso', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.2 0.2 0.4 0.6]);
plot3(X, Y, Z, 'b-', 'LineWidth', 3); hold on; grid on; axis equal;
plot3(P_global(1,:), P_global(2,:), P_global(3,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
title('Trayectoria Planificada Continuidad C^2', 'FontSize', 12);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z Profundidad [mm]');
set(gca, 'ZDir', 'reverse');
legend('Trayectoria Suavizada', 'Waypoints de Control', 'Location', 'best');
view(40, 30);

%% 10. PLANOS 2D DE LA TRAYECTORIA (XY, XZ, YZ)
fig3 = figure('Name', 'Planos 2D', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.8 0.4]);
subplot(1, 3, 1);
plot(X, Y, 'b-', 'LineWidth', 2); hold on; grid on;
plot(P_global(1,:), P_global(2,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
title('Plano XY (Vista Superior)'); xlabel('X [mm]'); ylabel('Y [mm]'); axis equal;
subplot(1, 3, 2);
plot(X, Z, 'r-', 'LineWidth', 2); hold on; grid on;
plot(P_global(1,:), P_global(3,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
title('Plano XZ (Vista Frontal)'); xlabel('X [mm]'); ylabel('Z [mm]');
set(gca, 'YDir', 'reverse'); axis equal;
subplot(1, 3, 3);
plot(Y, Z, 'g-', 'LineWidth', 2); hold on; grid on;
plot(P_global(2,:), P_global(3,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
title('Plano YZ (Vista Lateral)'); xlabel('Y [mm]'); ylabel('Z [mm]');
set(gca, 'YDir', 'reverse'); axis equal;

%% 11. EVOLUCIÓN DEL RADIO DE CURVATURA
fig4 = figure('Name', 'Radio de Curvatura Físico', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.3 0.2 0.4 0.45]);
Radio_Evolucion = 1 ./ (Curvatura + 1e-12);
plot(Z, Radio_Evolucion, 'm-', 'LineWidth', 2.5); hold on; grid on;
yline(R_min, 'r--', 'Límite Físico del BHA (R_{min})', 'LineWidth', 2, 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom');
xline(L_bit, 'k:', 'Fin Tramo Rígido', 'LabelHorizontalAlignment', 'left');
title('Evolución del Radio de Curvatura (\rho) vs Profundidad');
xlabel('Profundidad Z [mm]'); ylabel('Radio de Curvatura [mm]');
ylim([0, R_min*6]); xlim([0, max(Z)]);

%% 12. ANÁLISIS UNIFICADO DE AZIMUT (ABSOLUTO Y ACUMULADO)
fig5 = figure('Name', 'Trayectoria y Esfuerzo de Rotación', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.2 0.2 0.6 0.5]);
hold on; grid on;

% Configuramos el Eje Y Izquierdo (yyaxis left): Azimut Absoluto
yyaxis left
h1 = plot(Z, Phi_deg_absoluto, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 3, 'LineStyle', '-');
ylabel('Azimut Absoluto (\Phi) [Grados]', 'Color', [0.8500 0.3250 0.0980], 'FontSize', 11);
set(gca, 'YColor', [0.8500 0.3250 0.0980]); 

% Configuramos el Eje Y Derecho (yyaxis right): Azimut Acumulado
yyaxis right
h2 = plot(Z, Phi_deg_acumulado, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 3, 'LineStyle', '--');
ylabel('Esfuerzo de Rotación Total [\Sigma|\Delta\Phi|]', 'Color', [0.4940 0.1840 0.5560], 'FontSize', 11);
set(gca, 'YColor', [0.4940 0.1840 0.5560]); 

% Configuraciones compartidas
xlabel('Profundidad Z [mm]', 'FontSize', 11);
xline(L_bit, 'r--', 'Fin Tramo Rígido (88 mm)', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'middle');
xlim([0 max(Z)]);

xline(200, 'k:', 'P2 kick-off (200mm)');
xline(400, 'k:', 'P3 target (400mm)');

title({'Comparativa: Orientación Brújula vs Esfuerzo Mecánico'; 'Trayectoria Drillbotics Unificada - Arranque Limpio'}, 'FontSize', 13);
legend([h1, h2], {'Azimut Oficial (Arranca en 0°)', 'Esfuerzo Acumulado (Odómetro)'}, 'Location', 'best', 'FontSize', 10);