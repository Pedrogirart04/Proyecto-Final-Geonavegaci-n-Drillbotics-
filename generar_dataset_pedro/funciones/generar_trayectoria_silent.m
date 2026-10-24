function Matriz_Salida = generar_trayectoria_silent(P1, P2, P3)
    %% =========================================================================
    % FUNCIÓN GENERADORA DE TRAYECTORIA CONTÍNUA PARA SIMULINK (TRAMO DINÁMICO)
    % =========================================================================
    
    % 1. PARÁMETROS FÍSICOS Y WAYPOINTS BÁSICOS [EN MILÍMETROS]
    R_min = 196.65; % Radio de curvatura mínimo admisible [mm]
    
    P0 = [0, 0, 0];                
    P_global = [P0; P1; P2; P3]'; 
    
    % La curva calculada por spline arranca estrictamente en P1
    P_curve = [P1; P2; P3]'; 
    
    % Calculamos la longitud del tramo recto inicial de forma dinámica
    L_recto = norm(P1 - P0); 
    
    % 2. PARAMETRIZACIÓN POR DISTANCIA ACUMULADA (SOLO TRAMO CURVO)
    dists_curve = sqrt(sum(diff(P_curve, 1, 2).^2, 1));
    t_waypoints_curve = [0, cumsum(dists_curve)];
    
    % La distancia total contempla el tramo recto inicial dinámico + la curva
    Total_S = L_recto + t_waypoints_curve(end);
    N_puntos = 1200;
    t_fino = linspace(0, Total_S, N_puntos);
    
    % 3. INTERPOLACIÓN MIXTA (RECTA PURA P0->P1 + SPLINE SUJETO P1->P3)
    X = zeros(1, N_puntos);
    Y = zeros(1, N_puntos);
    Z = zeros(1, N_puntos);
    
    % Separamos los índices para cada tramo
    idx_recto = t_fino <= L_recto;
    idx_curvo = t_fino > L_recto;
    
    % TRAMO RECTO INICIAL: Interpolación lineal pura entre P0 y P1
    X(idx_recto) = linspace(P0(1), P1(1), sum(idx_recto));
    Y(idx_recto) = linspace(P0(2), P1(2), sum(idx_recto));
    Z(idx_recto) = linspace(P0(3), P1(3), sum(idx_recto));
    
    % TRAMO CURVO: La spline opera únicamente desde P1 en adelante
    v_start = (P1 - P0)' / L_recto; 
    v_end = (P_curve(:,end) - P_curve(:,end-1)) / dists_curve(end);
    
    t_fino_curvo = t_fino(idx_curvo) - L_recto;
    trayectoria_curva = zeros(3, sum(idx_curvo));
    
    for coord = 1:3
        y_interp = [v_start(coord), P_curve(coord,:), v_end(coord)];
        trayectoria_curva(coord, :) = spline(t_waypoints_curve, y_interp, t_fino_curvo);
    end
    
    X(idx_curvo) = trayectoria_curva(1, :);
    Y(idx_curvo) = trayectoria_curva(2, :);
    Z(idx_curvo) = trayectoria_curva(3, :);
    
    % 4. VARIABLES DE CONTROL: CINEMÁTICA Y GRADIENTES AISLADOS
    dX = zeros(1, N_puntos); dY = zeros(1, N_puntos); dZ = zeros(1, N_puntos);
    
    % Tramo recto (Derivada constante, velocidad lineal)
    dX(idx_recto) = (P1(1) - P0(1)) / L_recto;
    dY(idx_recto) = (P1(2) - P0(2)) / L_recto;
    dZ(idx_recto) = (P1(3) - P0(3)) / L_recto;
    
    % Tramo curvo (Derivadas numéricas aisladas)
    dX(idx_curvo) = gradient(X(idx_curvo), t_fino(idx_curvo));
    dY(idx_curvo) = gradient(Y(idx_curvo), t_fino(idx_curvo));
    dZ(idx_curvo) = gradient(Z(idx_curvo), t_fino(idx_curvo));
    
    ds = sqrt(dX.^2 + dY.^2 + dZ.^2);
    S_perforada = [0, cumsum(ds(1:end-1) .* diff(t_fino))];
    Theta_ref = acos(dZ ./ ds);
    Theta_deg = Theta_ref * (180/pi); 
    
    % --- CÁLCULO DE INCLINACIÓN ACUMULADA ---
    delta_theta_deg = diff(Theta_deg);
    Theta_deg_acumulado = [0, cumsum(abs(delta_theta_deg))];
    
    % 5. CÁLCULO DE AZIMUT POR VECTORES SECANTES (MÉTODO CONRA + FIX BLINDADO)
    k = 10; 
    Phi_raw = zeros(1, N_puntos);
    
    for i = 1:N_puntos
        idx_atras = max(1, i-k);
        idx_adelante = min(N_puntos, i+k);
        vec_X = X(idx_adelante) - X(idx_atras);
        vec_Y = Y(idx_adelante) - Y(idx_atras);
        Phi_raw(i) = atan2(vec_Y, vec_X);
    end
    
    Angulo_Salida_Ideal = atan2(P_global(2,4) - P_global(2,3), P_global(1,4) - P_global(1,3)); 
    idx_z100 = find(Z >= 100, 1, 'first'); 
    idx_confiable = find(Z > 200 & abs(Phi_raw - Angulo_Salida_Ideal) < 0.05, 1, 'first');
    if isempty(idx_confiable)
        idx_confiable = find(Z >= 250, 1, 'first'); 
    end
    
    Phi_raw(1:idx_z100) = 0;
    Phi_raw(idx_z100+1:idx_confiable) = Angulo_Salida_Ideal;
    Phi_raw = unwrap(Phi_raw);
    
    Phi_sin_montana = smoothdata(Phi_raw, 'movmedian', 100); 
    Phi_oficial_rad = smoothdata(Phi_sin_montana, 'gaussian', 100); 
    Phi_deg_absoluto = mod(Phi_oficial_rad * (180/pi), 360);
    
    delta_phi_deg = diff(Phi_oficial_rad * (180/pi));
    Phi_deg_acumulado = [0, cumsum(abs(delta_phi_deg))];
    
    % 6. ANÁLISIS DE CURVATURA ANALÍTICA 3D (2DA DERIVADA AISLADA)
    ddX = zeros(1, N_puntos); ddY = zeros(1, N_puntos); ddZ = zeros(1, N_puntos);
    
    % El tramo recto queda con 2da derivada en 0 por defecto (recta infinita)
    % Tramo curvo: calculamos la aceleración numéricamente aislada
    ddX(idx_curvo) = gradient(dX(idx_curvo), t_fino(idx_curvo));
    ddY(idx_curvo) = gradient(dY(idx_curvo), t_fino(idx_curvo));
    ddZ(idx_curvo) = gradient(dZ(idx_curvo), t_fino(idx_curvo));
    
    Numerador_K = sqrt((dY.*ddZ - dZ.*ddY).^2 + (dZ.*ddX - dX.*ddZ).^2 + (dX.*ddY - dY.*ddX).^2);
    Curvatura = Numerador_K ./ (ds.^3);
    
    % Evitamos matemática sucia o divisiones por cero en el tramo recto
    Curvatura(idx_recto) = 0;
    
    Max_Curvatura = max(Curvatura(idx_curvo)); 
    Min_Radio_Real = 1 / (Max_Curvatura + 1e-12);
    
    Radio_Evolucion = 1 ./ (Curvatura + 1e-12);
    Radio_Evolucion(Radio_Evolucion > 10000) = 10000; 
    
    % 7. CONSTRUCCIÓN DE MATRIZ DE SALIDA (N x 6)
    Matriz_Salida = [X(:), Y(:), Z(:), Radio_Evolucion(:), Theta_deg(:), Phi_deg_absoluto(:)];
    
    % 8. REPORTE EN CONSOLA
    % fprintf('==================================================\n');
    % fprintf('     REPORTE DE TRAYECTORIA (ARRANQUE DINÁMICO)   \n');
    % fprintf('==================================================\n');
    % fprintf(' Longitud total del pozo (S):     %.2f mm\n', S_perforada(end));
    % fprintf(' Radio de curvatura mínimo real:  %.2f mm\n', Min_Radio_Real);
    % fprintf(' Esfuerzo Inclinación total:      %.2f°\n', Theta_deg_acumulado(end));
    % fprintf(' Esfuerzo de rotación total:      %.2f°\n', Phi_deg_acumulado(end));
    % fprintf('--------------------------------------------------\n');
    % if Min_Radio_Real < R_min
    %     fprintf(2, ' [!] ALERTA CRÍTICA: El radio infringe el límite de %0.2f mm.\n', R_min);
    % else
    %     disp(' [OK] Radio seguro. Señales numéricas limpias.');
    % end
    % fprintf('==================================================\n');
    
end