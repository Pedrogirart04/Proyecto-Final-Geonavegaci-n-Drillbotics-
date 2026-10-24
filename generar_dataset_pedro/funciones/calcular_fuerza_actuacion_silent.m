function Matriz_Resultados = calcular_fuerza_actuacion_silent(Matriz_Trayectoria, F2, F3, R_min, R_umb, l_a, l_b, d_a, d_b, g, coef_I)
    %% =========================================================================
    % FUNCION REFACTURADA: EVALUACIÓN MECÁNICA Y ELÉCTRICA CON F_LAT EXTERNA
    % =========================================================================
    
    % 1. Extracción del Radio de Curvatura de la trayectoria
    R = Matriz_Trayectoria(:, 4); 
    
    % 2. Mapeo lineal de los brazos de palanca respecto al radio 'R'
    l_var = l_a + (l_b - l_a) * (R - R_min) / (R_umb - R_min);
    l_var(R < R_min) = l_a; 
    l_var(R > R_umb) = l_b; 
    
    d_var = d_a + (d_b - d_a) * (R - R_min) / (R_umb - R_min);
    d_var(R < R_min) = d_a; 
    d_var(R > R_umb) = d_b; 
    
    % 3. Relación del mecanismo: Fc = Fp * (d / l) usando la fuerza externa
    % Aseguramos que F_lat_mag sea un vector columna/fila coincidente
    F_lat_mag = sqrt(F2.^2+F3.^2);
    F_cable_mag = F_lat_mag(:) .* (d_var(:) ./ l_var(:));
    
    % 4. Conversión a Corriente usando el polinomio empírico de calibración
    Masa_eq_kg = F_cable_mag / g; 
    Corriente_A = polyval(coef_I, Masa_eq_kg);
    Corriente_A(Corriente_A < 0) = 0; % Filtrar corrientes negativas del servo
    
    % 5. Armado de matriz de salida
    Matriz_Resultados = [R(:), F_lat_mag(:), l_var(:), d_var(:), F_cable_mag(:), Corriente_A(:)];

    % Extracción para graficar
    R_vector       = Matriz_Resultados(:, 1);
    F_pared        = Matriz_Resultados(:, 2);
    L_brazo_vector = Matriz_Resultados(:, 3);
    D_brazo_vector = Matriz_Resultados(:, 4);
    F_cable        = Matriz_Resultados(:, 5);
    Corriente_A    = Matriz_Resultados(:, 6);
    Z_global       = Matriz_Trayectoria(:, 3); 
    
end