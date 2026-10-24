function generar_figura_esquema_pads()
%% =========================================================================
% GENERADOR DE ESQUEMA TÉCNICO: DISPOSICIÓN DE PADS EN EL BHA
% Proyecto Drillbotics - Tesis de Geonavegación
% =========================================================================

    % 1. Configuración de la Figura
    fig = figure('Name', 'Esquema Técnico: Disposición de Pads', 'Color', 'w', ...
                 'Units', 'normalized', 'Position', [0.3 0.3 0.4 0.5]);
    
    ax = axes('Parent', fig);
    hold(ax, 'on'); 
    axis(ax, 'equal');
    grid(ax, 'off'); 
    set(ax, 'XLim', [-1.6 1.8], 'YLim', [-1.6 1.6], 'Visible', 'on');
    
    title(ax, 'Esquema de Distribución Angular de Actuadores (Pads)', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 2. Parámetros Geométricos Simulados
    R_bha = 1.0;            
    Thick_pad = 0.15;       
    Width_pad_deg = 40;     
    
    angles_deg = [0, 120, 240];
    colors = {[0.6350, 0.0780, 0.1840], [0.4660, 0.6740, 0.1880], [0.8500, 0.3250, 0.0980]}; % RGB Compatibles
    
    %% 3. Dibujar Ejes de Referencia (Locales/Globales)
    % Eje X 
    line(ax, [-1.5 1.7], [0 0], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.0);
    text(ax, 1.6, 0.08, '\textbf{X}', 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
    
    % Eje Y
    line(ax, [0 0], [-1.5 1.5], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.0);
    text(ax, 0.08, 1.4, '\textbf{Y}', 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
    
    %% 4. Dibujar el Cuerpo Central del BHA
    theta_circle = linspace(0, 2*pi, 150);
    X_bha = R_bha * cos(theta_circle);
    Y_bha = R_bha * sin(theta_circle);
    
    fill(ax, X_bha, Y_bha, [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 2.0);
    
    plot(ax, 0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 1.5);
    %text(ax, -0.2, -0.15, 'BHA', 'FontSize', 10, 'FontWeight', 'bold');
    
    %% 5. Dibujar los Pads y sus Etiquetas Radiales
    for i = 1:3
        ang_center = angles_deg(i);
        
        % Coordenadas para rellenar el pad
        theta_start = (ang_center - Width_pad_deg/2) * (pi/180);
        theta_end = (ang_center + Width_pad_deg/2) * (pi/180);
        theta_pad = linspace(theta_start, theta_end, 50);
        
        X_inner = R_bha * cos(theta_pad);
        Y_inner = R_bha * sin(theta_pad);
        X_outer = (R_bha + Thick_pad) * cos(fliplr(theta_pad));
        Y_outer = (R_bha + Thick_pad) * sin(fliplr(theta_pad));
        
        X_fill = [X_inner, X_outer, X_inner(1)];
        Y_fill = [Y_inner, Y_outer, Y_inner(1)];
        
        fill(ax, X_fill, Y_fill, colors{i}, 'EdgeColor', 'k', 'LineWidth', 1.5);
        
        %% 6. Agregar Líneas Radiales y Texto Limpio
        % Línea de trazos marcando el eje radial del pad
        R_line = R_bha;
        line(ax, [0 R_line*cos(ang_center*pi/180)], [0 R_line*sin(ang_center*pi/180)], ...
             'Color', colors{i}, 'LineStyle', ':', 'LineWidth', 1.2);
        
        % Posición del texto un poquito más allá de la línea
        R_text = R_bha + Thick_pad + 0.05;
        tx = R_text * cos(ang_center*pi/180);
        ty = R_text * sin(ang_center*pi/180);
        
        % Alineación para que no pise las líneas
        if ang_center == 0
            ha = 'left'; va = 'middle';
        elseif ang_center == 120
            ha = 'right'; va = 'bottom';
        elseif ang_center == 240
            ha = 'right'; va = 'top';
        end
        
        % Armamos el texto en dos renglones (Ej: "Pad 1" y abajo "0°")
        label_str = {sprintf('Pad %d', i), sprintf('%d^\\circ', ang_center)};
        
        text(ax, tx, ty, label_str, 'Color', colors{i}, 'FontSize', 11, ...
             'FontWeight', 'bold', 'HorizontalAlignment', ha, 'VerticalAlignment', va, ...
             'Interpreter', 'tex');
    end
    
    %% 7. Configuración final
    set(ax, 'XTick', [], 'YTick', []); % Apaga los números de los bordes
end