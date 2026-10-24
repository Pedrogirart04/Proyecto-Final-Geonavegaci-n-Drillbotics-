%% =========================================================================
% SCRIPT MAIN - CÁLCULO DE FUERZA DE CABLE BHA (BRAZOS DINÁMICOS Y CORRIENTE)
% =========================================================================
clear; clc; close all;

%% 1. Definición de los Waypoints objetivo [X, Y, Z] en mm
P2 = [0, 0, 200];        
P3 = [40, 5, 400];       
P4 = [50, 10, 600];       

%% 2. Generación de Trayectoria
Matriz_Trayectoria = generar_trayectoria(P2, P3, P4);

%% 3. PARÁMETROS OPERATIVOS Y MECÁNICOS
ROP_mm_min = 20;  
RPM = 800;        
g = 9.81; % [m/s^2] Aceleración de la gravedad para conversión N a Kg

% Propiedades de la roca y geometría del trépano
params_roca.epsilon = 25; 
params_roca.zeta_f  = 1;  
params_roca.zeta_g  = 10; 
params_trepano.a = 1.5*25.4/2; 
params_trepano.b = 5; 

% --- PARÁMETROS DEL MECANISMO DE DIRECCIÓN (DCL DINÁMICO) ---
% Radios de control
R_min_bha = 186.43; % [mm] Radio de curvatura límite del BHA
R_umbral  = 1000;   % [mm] Radio a partir del cual se considera trayectoria recta

% Brazos de palanca variables del cable (distancia 'l')
l_a = 7.27; % [mm] Brazo de palanca cuando R = R_min_bha 
l_b = 4.11; % [mm] Brazo de palanca cuando R >= R_umbral 

% Brazos de palanca variables del empuje (distancia 'd')
d_a = 10.8;   % [mm] Brazo de palanca cuando R = R_min_bha 
d_b = 13.88;  % [mm] Brazo de palanca cuando R >= R_umbral 

% --- COEFICIENTES POLINOMIO EXCEL (I vs Kg) ---
coef_I = [1.55957502e-04, 3.33106612e-03, 2.37648267e-02, 5.90535677e-02];

%% 4. CÁLCULO DE FUERZAS Y ARMADO DE MATRIZ
Matriz_Resultados = calcular_fuerza_actuacion(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano, R_min_bha, R_umbral, l_a, l_b, d_a, d_b, g, coef_I);

% Extracción para graficar
R_vector       = Matriz_Resultados(:, 1);
F_pared        = Matriz_Resultados(:, 2);
L_brazo_vector = Matriz_Resultados(:, 3);
D_brazo_vector = Matriz_Resultados(:, 4);
F_cable        = Matriz_Resultados(:, 5);
Corriente_A    = Matriz_Resultados(:, 6);
Z_global       = Matriz_Trayectoria(:, 3); 

%% 5. REPORTES Y GRÁFICOS DEL MECANISMO
fprintf('==================================================\n');
fprintf(' RESUMEN DE CARGAS Y CONSUMO ELÉCTRICO\n');
fprintf('==================================================\n');
fprintf(' Fuerza máxima en la pared: %.2f N\n', max(F_pared));
fprintf(' Tensión máxima en el cable: %.2f N\n', max(F_cable));
fprintf(' Corriente pico requerida: %.3f A\n', max(Corriente_A));
fprintf('==================================================\n');

fig_mecanismo = figure('Name', 'Análisis de Actuación del Cable', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.8 0.8]);

% Subplot 1: Evolución de los brazos de palanca (l y d)
subplot(2,2,1);
plot(Z_global, L_brazo_vector, 'b', 'LineWidth', 2); hold on; grid on;
plot(Z_global, D_brazo_vector, 'm', 'LineWidth', 2);
xlabel('Profundidad Z [mm]'); ylabel('Brazo de palanca [mm]');
title('Evolución Geométrica');
legend('Brazo Cable (l)', 'Brazo Empuje (d)', 'Location', 'best');
set(gca, 'XDir', 'reverse');

% Subplot 2: Fuerza Lateral en la Pared
subplot(2,2,2);
plot(Z_global, F_pared, 'Color', '#D95319', 'LineWidth', 2); grid on;
xlabel('Profundidad Z [mm]'); ylabel('Fuerza en la Pared (F_{lat}) [N]');
title('Requerimiento de Empuje');
set(gca, 'XDir', 'reverse');

% Subplot 3: Tensión resultante en el Cable
subplot(2,2,3);
plot(Z_global, F_cable, 'Color', '#7E2F8E', 'LineWidth', 2.5); grid on;
xlabel('Profundidad Z [mm]'); ylabel('Tensión del Cable (F_c) [N]');
title('Esfuerzo de Tracción en Cable');
set(gca, 'XDir', 'reverse');

% Subplot 4: Consumo de Corriente del Servomotor
subplot(2,2,4);
plot(Z_global, Corriente_A, 'Color', '#77AC30', 'LineWidth', 2.5); grid on;
xlabel('Profundidad Z [mm]'); ylabel('Corriente [A]');
title('Consumo Eléctrico Estimado del Servo');
set(gca, 'XDir', 'reverse');

%% =========================================================================
% FUNCIONES LOCALES
% =========================================================================

function Matriz_Resultados = calcular_fuerza_actuacion(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano, R_min, R_umb, l_a, l_b, d_a, d_b, g, coef_I)
    % 1. Extracción de R
    R = Matriz_Trayectoria(:, 4); 
    
    % 2. Mapeo lineal de los brazos de palanca respecto al radio 'R'
    l_var = l_a + (l_b - l_a) * (R - R_min) / (R_umb - R_min);
    l_var(R < R_min) = l_a; 
    l_var(R > R_umb) = l_b; 
    
    d_var = d_a + (d_b - d_a) * (R - R_min) / (R_umb - R_min);
    d_var(R < R_min) = d_a; 
    d_var(R > R_umb) = d_b; 
    
    % 3. Cálculo de la Fuerza Lateral (Perneder Reducido)
    eps = params_roca.epsilon; zf = params_roca.zeta_f; zg = params_roca.zeta_g;
    a = params_trepano.a; b = params_trepano.b; nu = b / a;
    
    d1 = (ROP_mm_min / RPM) * ones(size(R)); 
    phi_3 = d1 ./ R; 
    d2 = zeros(size(R)); d3 = zeros(size(R));
    
    idx_curva = R < R_umb; 
    d2(idx_curva) = 0.05 .* (R_min ./ R(idx_curva)); 
    d3(idx_curva) = 0.02 .* (R_min ./ R(idx_curva)); 
    
    d_lateral_mag = sqrt(d2.^2 + d3.^2);
    psi = zeros(size(R));
    idx_valido = d1 > 0;
    psi(idx_valido) = -d_lateral_mag(idx_valido) ./ d1(idx_valido);  
    psi_star = 2 * b ./ (R + 1e-9); 
    
    Psi_1 = ones(size(R)); Psi_2 = ones(size(R));
    ratio = zeros(size(R));
    ratio(idx_curva) = psi(idx_curva) ./ psi_star(idx_curva);
    
    idx_C2 = (ratio >= -0.5) & (ratio < 0);
    Psi_1(idx_C2) = 1 + 3 * ratio(idx_C2);
    Psi_2(idx_C2) = 1 - 3 * (ratio(idx_C2).^2);
    
    idx_C3 = (ratio >= -1.0) & (ratio < -0.5);
    Psi_1(idx_C3) = -ratio(idx_C3);
    Psi_2(idx_C3) = ratio(idx_C3).^2;
    
    L22 = eps * a * zg * nu .* Psi_1;
    L24 = -eps * a^2 * zg * nu^2 .* Psi_2;
    
    F2 = (L22 .* d2) + (L24 .* phi_3);
    F3 = (L22 .* d3);
    
    F_lat_mag = sqrt(F2.^2 + F3.^2); 
    
    % 4. Relación del mecanismo: Fc = Fp * (d / l) con variables dinámicas
    F_cable_mag = F_lat_mag .* (d_var ./ l_var);
    
    % 5. Conversión a Corriente usando el polinomio empírico
    Masa_eq_kg = F_cable_mag / g; 
    Corriente_A = polyval(coef_I, Masa_eq_kg);
    Corriente_A(Corriente_A < 0) = 0; % Filtrar corrientes negativas
    
    % 6. Armado de matriz de salida
    Matriz_Resultados = [R(:), F_lat_mag(:), l_var(:), d_var(:), F_cable_mag(:), Corriente_A(:)];
end

function Matriz_Salida = generar_trayectoria(P2, P3, P4)
    %% =========================================================================
    % FUNCIÓN GENERADORA DE TRAYECTORIA CONTÍNUA PARA SIMULINK
    % =========================================================================
    
    % 1. PARÁMETROS FÍSICOS Y WAYPOINTS BÁSICOS [EN MILÍMETROS]
    L_bit = 88;     % Largo de la sección rígida inicial [mm]
    R_min = 186.43; % Radio de curvatura mínimo admisible [mm]
    
    P0 = [0, 0, 0];          
    P1 = [0, 0, L_bit];      
    P_global = [P0; P1; P2; P3; P4]'; 
    
    % 2. PARAMETRIZACIÓN POR DISTANCIA ACUMULADA
    dists = sqrt(sum(diff(P_global, 1, 2).^2, 1));
    t_waypoints = [0, cumsum(dists)];
    N_puntos = 1200;
    t_fino = linspace(0, t_waypoints(end), N_puntos);
    
    % 3. INTERPOLACIÓN POR SPLINE SUJETO
    v_start = [0; 0; 1];
    v_end = (P_global(:,end) - P_global(:,end-1)) / dists(end);
    trayectoria = zeros(3, N_puntos);
    for coord = 1:3
        y_interp = [v_start(coord), P_global(coord,:), v_end(coord)];
        trayectoria(coord, :) = spline(t_waypoints, y_interp, t_fino);
    end
    X = trayectoria(1, :); Y = trayectoria(2, :); Z = trayectoria(3, :);
    
    % 4. VARIABLES DE CONTROL: INCLINACIÓN (THETA)
    dX = gradient(X, t_fino); dY = gradient(Y, t_fino); dZ = gradient(Z, t_fino);
    ds = sqrt(dX.^2 + dY.^2 + dZ.^2);
    S_perforada = [0, cumsum(ds(1:end-1) .* diff(t_fino))];
    Theta_ref = acos(dZ ./ ds);
    Theta_deg = Theta_ref * (180/pi); % OBTENEMOS GRADOS AQUÍ
    
    % --- CÁLCULO DE INCLINACIÓN ACUMULADA ---
    delta_theta_deg = diff(Theta_deg);
    Theta_deg_acumulado = [0, cumsum(abs(delta_theta_deg))];
    
    % 5. CÁLCULO DE AZIMUT POR VECTORES SECANTES (MÉTODO CONRA + FIX BLINDADO)
    k = 10; % Usamos solo la ventana de caño rígido
    Phi_raw = zeros(1, N_puntos);
    
    for i = 1:N_puntos
        idx_atras = max(1, i-k);
        idx_adelante = min(N_puntos, i+k);
        vec_X = X(idx_adelante) - X(idx_atras);
        vec_Y = Y(idx_adelante) - Y(idx_atras);
        Phi_raw(i) = atan2(vec_Y, vec_X);
    end
    
    % Eliminación del Overshoot
    Angulo_Salida_Ideal = atan2(P_global(2,4) - P_global(2,3), P_global(1,4) - P_global(1,3)); 
    idx_z100 = find(Z >= 100, 1, 'first'); 
    idx_confiable = find(Z > 200 & abs(Phi_raw - Angulo_Salida_Ideal) < 0.05, 1, 'first');
    if isempty(idx_confiable)
        idx_confiable = find(Z >= 250, 1, 'first'); 
    end
    
    Phi_raw(1:idx_z100) = 0;
    Phi_raw(idx_z100+1:idx_confiable) = Angulo_Salida_Ideal;
    Phi_raw = unwrap(Phi_raw);
    
    % Filtro Doble Pasada
    Phi_sin_montana = smoothdata(Phi_raw, 'movmedian', 100); 
    Phi_oficial_rad = smoothdata(Phi_sin_montana, 'gaussian', 100); 
    
    % OBTENEMOS GRADOS Y NORMALIZAMOS DE 0 A 360 AQUÍ
    Phi_deg_absoluto = mod(Phi_oficial_rad * (180/pi), 360);
    
    % --- CÁLCULO DE AZIMUT ACUMULADO ---
    delta_phi_deg = diff(Phi_oficial_rad * (180/pi));
    Phi_deg_acumulado = [0, cumsum(abs(delta_phi_deg))];
    
    % 6. ANÁLISIS DE CURVATURA ANALÍTICA 3D 
    ddX = gradient(dX, t_fino); ddY = gradient(dY, t_fino); ddZ = gradient(dZ, t_fino);
    Numerador_K = sqrt((dY.*ddZ - dZ.*ddY).^2 + (dZ.*ddX - dX.*ddZ).^2 + (dX.*ddY - dY.*ddX).^2);
    Curvatura = Numerador_K ./ (ds.^3);
    Max_Curvatura = max(Curvatura(t_fino > L_bit)); 
    Min_Radio_Real = 1 / (Max_Curvatura + 1e-12);
    
    Radio_Evolucion = 1 ./ (Curvatura + 1e-12);
    Radio_Evolucion(Radio_Evolucion > 10000) = 10000; % Tope de 10 metros para la recta
    
    % 7. CONSTRUCCIÓN DE MATRIZ DE SALIDA (N x 6)
    Matriz_Salida = [X(:), Y(:), Z(:), Radio_Evolucion(:), Theta_deg(:), Phi_deg_absoluto(:)];
    
   % 9. GRÁFICOS
    fig2 = figure('Name', 'Trayectoria 3D liso', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.2 0.2 0.4 0.6]);
    plot3(X, Y, Z, 'b-', 'LineWidth', 3); hold on; grid on; axis equal;
    plot3(P_global(1,:), P_global(2,:), P_global(3,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
    title('Trayectoria Planificada', 'FontSize', 12);
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z Profundidad [mm]');
    set(gca, 'ZDir', 'reverse');
    legend('Trayectoria Suavizada', 'Waypoints de Control', 'Location', 'best');
    view(40, 30);
      
    fig4 = figure('Name', 'Radio de Curvatura Físico', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.3 0.2 0.4 0.45]);
    plot(Z, Radio_Evolucion, 'm-', 'LineWidth', 2.5); hold on; grid on;
    yline(R_min, 'r--', 'Límite Físico del BHA (R_{min})', 'LineWidth', 2, 'LabelHorizontalAlignment', 'center', 'LabelVerticalAlignment', 'bottom');
    xline(L_bit, 'k:', 'Fin Tramo Rígido', 'LabelHorizontalAlignment', 'left');
    title('Evolución del Radio de Curvatura (\rho) vs Profundidad');
    xlabel('Profundidad Z [mm]'); ylabel('Radio de Curvatura [mm]');
    ylim([0, R_min*6]); xlim([0, max(Z)]);
end