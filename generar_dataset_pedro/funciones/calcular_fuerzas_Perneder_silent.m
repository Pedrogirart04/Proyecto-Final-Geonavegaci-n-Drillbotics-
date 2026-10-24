function Matriz_Fuerzas_3D = calcular_fuerzas_Perneder_silent(Matriz_Trayectoria, ROP_mm_min, RPM, params_roca, params_trepano)
% =========================================================================
% CÁLCULO Y VISUALIZACIÓN 3D DE FUERZAS BIT-ROCA (PERNEDER 2012)
% Proyecto Drillbotics ITBA
% =========================================================================

    %% 1. EXTRACCIÓN DE DATOS DE TRAYECTORIA GLOBAL
    X_global = Matriz_Trayectoria(:, 1);
    Y_global = Matriz_Trayectoria(:, 2);
    Z_global = Matriz_Trayectoria(:, 3); % Profundidad Z [mm]
    R = Matriz_Trayectoria(:, 4); % Radio de curvatura 3D [mm]
    
    %% 2. PARÁMETROS FÍSICOS Y GEOMÉTRICOS
    eps   = params_roca.epsilon;    % Energía Específica (MPa o N/mm^2)
    zf    = params_roca.zeta_f;     % Coef. Interacción Frente
    zg    = params_roca.zeta_g;     % Coef. Interacción Calibre
    
    a     = params_trepano.a;       % Radio del trépano [mm]
    b     = params_trepano.b;       % Mitad de la altura del calibre [mm]
    nu    = b / a;                  % Esbeltez
    L_bit = params_trepano.L_bit;   % Importado para la gráfica 3D
    
    %% 3. CÁLCULO DE CINEMÁTICA 3D (PENETRACIONES LOCALES)
    % Penetración axial constante [mm/rev]
    d1 = (ROP_mm_min / RPM) * ones(size(R)); 
    
    % Penetración angular principal (flexión sobre eje i2)
    phi_3 = d1 ./ R; 
    phi_2 = zeros(size(R)); 
    
    % Penetraciones laterales inducidas progresivas [mm/rev]
    d2 = zeros(size(R));
    d3 = zeros(size(R));
    
    % Umbral de detección de curva (Ej: R < 1000 mm)
    idx_curva = R < 1000; 
    
    % Límite geométrico real del BHA (Drillbotics Scale)
    R_min_bha = 186.43; 
    
    % Empuje lateral progresivo variable
    % Calibrado para que a R_min el empuje sea máximo
    d2(idx_curva) = 0.05 .* (R_min_bha ./ R(idx_curva)); 
    d3(idx_curva) = 0.0 .* (R_min_bha ./ R(idx_curva)); 
    
    %% 4. FUNCIONES DE ESTADO Y TILT (LÓGICA C1-C4)
    d_lateral_mag = sqrt(d2.^2 + d3.^2);
    
    psi = zeros(size(R));
    idx_valido = d1 > 0;
    psi(idx_valido) = -d_lateral_mag(idx_valido) ./ d1(idx_valido);  
    
    psi_star = 2 * b ./ (R + 1e-9); 
    
    % Inicializamos todo en 1 (Configs C1 y C4 por defecto)
    Psi_1 = ones(size(R));
    Psi_2 = ones(size(R));
    Psi_3 = ones(size(R));
    
    ratio = zeros(size(R));
    ratio(idx_curva) = psi(idx_curva) ./ psi_star(idx_curva);
    
    % --- CONFIGURACIÓN C2 (Ratio entre -0.5 y 0) ---
    idx_C2 = (ratio >= -0.5) & (ratio < 0);
    Psi_1(idx_C2) = 1 + 3 * ratio(idx_C2);
    Psi_2(idx_C2) = 1 - 3 * (ratio(idx_C2).^2);
    Psi_3(idx_C2) = 1 + 7 * (ratio(idx_C2).^3);
    
    % --- CONFIGURACIÓN C3 (Ratio entre -1.0 y -0.5) ---
    idx_C3 = (ratio >= -1.0) & (ratio < -0.5);
    Psi_1(idx_C3) = -ratio(idx_C3);
    Psi_2(idx_C3) = ratio(idx_C3).^2;
    Psi_3(idx_C3) = -(ratio(idx_C3).^3);
    
    %% 5. MATRIZ DE INTERACCIÓN 3D LOCAL
    % Coeficientes base de Perneder
    L11 = eps * a * (pi/2);
    L22 = eps * a * zg * nu .* Psi_1;
    L24 = -eps * a^2 * zg * nu^2 .* Psi_2;
    L42 = 0.5 * eps * a^2 * zg * nu^2 .* Psi_2;
    L44 = -eps * a^3 * (zf/6 + (2/3)*zg*nu^3 .* Psi_3);
    
    % F1: Fuerza Axial [N]
    F1 = L11 .* d1;
    
    % Magnitudes locales de F2, F3, M2, M3
    F2 = (L22 .* d2) + (L24 .* phi_3);
    M3_Nmm = (L42 .* d2) + (L44 .* phi_3);
    F3 = (L22 .* d3) - (L24 .* phi_2);
    M2_Nmm = -(L42 .* d3) + (L44 .* phi_2);
         
    M2_Nm = M2_Nmm / 1000;
    M3_Nm = M3_Nmm / 1000;
    
    % Magnitud Resultante Lateral Total
    F_lat_mag = sqrt(F2.^2 + F3.^2);
    
    Matriz_Fuerzas_3D = [F1, F2, F3, M2_Nm, M3_Nm];
    
    %% 6. CÁLCULO DEL TRIEDRO DE FRENET-SERRET GLOBAL (Para Visualización)
    Points_XYZ = [X_global, Y_global, Z_global];
    
    % Tangente T (i1 globalizado)
    T = zeros(size(Points_XYZ));
    diff_P = diff(Points_XYZ);
    diff_P = [diff_P; diff_P(end,:)]; 
    mag_diff = sqrt(sum(diff_P.^2, 2)) + 1e-9;
    T = diff_P ./ mag_diff;

    % Binormal B (i3 globalizado)
    diff_T = diff(T);
    diff_T = [diff_T; diff_T(end,:)]; 
    B_raw = cross(T, diff_T);
    mag_B = sqrt(sum(B_raw.^2, 2)) + 1e-9;
    B = B_raw ./ mag_B;

    % Normal N (i2 globalizado)
    N = cross(B, T);

    %% 7. CONSTRUCCIÓN DE VECTORES DE FUERZA LATERAL GLOBALIZADOS
    F_XYZ_Global = zeros(size(T));
    F_XYZ_Global(:,1) = F2 .* N(:,1) + F3 .* B(:,1); 
    F_XYZ_Global(:,2) = F2 .* N(:,2) + F3 .* B(:,2); 
    F_XYZ_Global(:,3) = F2 .* N(:,3) + F3 .* B(:,3); 
end