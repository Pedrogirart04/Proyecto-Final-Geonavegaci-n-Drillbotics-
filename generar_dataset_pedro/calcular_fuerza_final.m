function Matriz_Resultados = calcular_fuerza_final(Matriz_Trayectoria, F_lat, R_min, R_umb, l_a, l_b, d_a, d_b)
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
    F_cable_mag = F_lat(:) .* (d_var(:) ./ l_var(:));
     
    % 5. Armado de matriz de salida
    Matriz_Resultados = [R(:), l_var(:), d_var(:), F_cable_mag(:)];

    F_cable        = Matriz_Resultados(:, 4);
    Z_global       = Matriz_Trayectoria(:, 3); 
    
    figure('Name', 'Análisis de Actuación del Cable', 'Color', 'w', 'Units', 'normalized');
    plot(Z_global, F_cable, 'Color', '#7E2F8E', 'LineWidth', 2.5); grid on;
    xlabel('Profundidad Z [mm]'); ylabel('Tensión del Cable (F_c) [N]');
    title('Esfuerzo de Tracción en Cable considerando la sarta');
    set(gca, 'XDir', 'normal');
end