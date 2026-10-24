function Matriz_Resultados = calcular_fuerza_actuacion(Matriz_Trayectoria, F2, F3, R_min, R_umb, l_a, l_b, d_a, d_b, g, coef_I)
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
    set(gca, 'XDir', 'normal');
    
    % Subplot 2: Fuerza Lateral en la Pared
    subplot(2,2,2);
    plot(Z_global, F_pared, 'Color', '#D95319', 'LineWidth', 2); grid on;
    xlabel('Profundidad Z [mm]'); ylabel('Fuerza en la Pared (F_{lat}) [N]');
    title('Requerimiento de Empuje');
    set(gca, 'XDir', 'normal');
    
    % Subplot 3: Tensión resultante en el Cable
    subplot(2,2,3);
    plot(Z_global, F_cable, 'Color', '#7E2F8E', 'LineWidth', 2.5); grid on;
    xlabel('Profundidad Z [mm]'); ylabel('Tensión del Cable (F_c) [N]');
    title('Esfuerzo de Tracción en Cable');
    set(gca, 'XDir', 'normal');
    
    % Subplot 4: Consumo de Corriente del Servomotor
    subplot(2,2,4);
    plot(Z_global, Corriente_A, 'Color', '#77AC30', 'LineWidth', 2.5); grid on;
    xlabel('Profundidad Z [mm]'); ylabel('Corriente [A]');
    title('Consumo Eléctrico Estimado del Servo');
    set(gca, 'XDir', 'normal');
end