function simulador_pads_interactivo()
%% =========================================================================
% SIMULADOR INTERACTIVO PUSH-THE-BIT (TESIS DRILLBOTICS)
% Mové el mouse sobre el gráfico para ver la activación de los pads
% =========================================================================

    % 1. Crear figura interactiva
    fig = figure('Name', 'Simulador Interactivo: Lógica Push-the-Bit', 'Color', 'w', ...
                 'WindowButtonMotionFcn', @mouse_move, ...
                 'Units', 'normalized', 'Position', [0.2 0.2 0.5 0.6]);
    
    % 2. Configurar ejes principales
    ax = axes('Parent', fig, 'XLim', [-1.5 1.5], 'YLim', [-1.5 1.5], ...
              'DataAspectRatio', [1 1 1]);
    hold(ax, 'on'); grid(ax, 'on');
    title(ax, 'Mové el mouse en el círculo para cambiar la dirección del BHA', 'FontSize', 12);
    
    % 3. Dibujar la pared del pozo (Isocurva de Fuerza Unitaria)
    theta_circle = linspace(0, 2*pi, 100);
    plot(ax, cos(theta_circle), sin(theta_circle), 'k--', 'LineWidth', 1.5);
    
    % 4. Dibujar herramienta central (BHA)
    plot(ax, 0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', '#7E2F8E');
    
    % 5. Inicializar Flechas (Vectores)
    % Vector de movimiento deseado (Hacia dónde queremos ir)
    h_mov = quiver(ax, 0, 0, 0, 0, 'Color', 'k', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    
    % Vectores de los Pads empujando la pared
    h_pad1 = quiver(ax, 0, 0, 0, 0, 'Color', 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5);
    h_pad2 = quiver(ax, 0, 0, 0, 0, 'Color', 'g', 'LineWidth', 4, 'MaxHeadSize', 0.5);
    h_pad3 = quiver(ax, 0, 0, 0, 0, 'Color', 'b', 'LineWidth', 4, 'MaxHeadSize', 0.5);
    
    % Ejes de referencia de los pads (Gris clarito)
    plot(ax, [0 1.5*cos(0)], [0 1.5*sin(0)], 'k:', 'Color', [0.7 0.7 0.7]);
    plot(ax, [0 1.5*cos(2*pi/3)], [0 1.5*sin(2*pi/3)], 'k:', 'Color', [0.7 0.7 0.7]);
    plot(ax, [0 1.5*cos(4*pi/3)], [0 1.5*sin(4*pi/3)], 'k:', 'Color', [0.7 0.7 0.7]);
    
    % 6. Panel de Texto dinámico
    h_text = text(ax, -1.4, 1.2, '', 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', 'k');

    % =====================================================================
    % FUNCIÓN CALLBACK (Se ejecuta cada vez que movés el mouse)
    % =====================================================================
    function mouse_move(~, ~)
        % Obtener coordenadas actuales del mouse en el gráfico
        C = get(ax, 'CurrentPoint');
        x = C(1,1);
        y = C(1,2);
        
        % Calcular radio y abortar si está en el centro exacto
        r = norm([x, y]);
        if r == 0; return; end
        
        % Forzamos que la magnitud máxima sea 1 (Isocurva de Fuerza Unitaria)
        F = min(r, 1); 
        x_norm = (x/r)*F;
        y_norm = (y/r)*F;
        
        % Ángulo de movimiento (Azimut simulado)
        theta_mov = atan2(y, x);
        if theta_mov < 0; theta_mov = theta_mov + 2*pi; end
        
        % LÓGICA PUSH-THE-BIT EXACTA (Copiada de tu Main)
        theta_push = mod(theta_mov + pi, 2*pi);
        th = theta_push;
        
        T1 = 0; T2 = 0; T3 = 0;
        
        if th >= 0 && th < (2*pi/3)
            T1 = F * (cos(th) + (1/sqrt(3))*sin(th));
            T2 = F * ((2/sqrt(3))*sin(th));
        elseif th >= (2*pi/3) && th < (4*pi/3)
            T2 = F * (-cos(th) + (1/sqrt(3))*sin(th));
            T3 = F * (-cos(th) - (1/sqrt(3))*sin(th));
        else
            T1 = F * (cos(th) - (1/sqrt(3))*sin(th));
            T3 = F * (-(2/sqrt(3))*sin(th));
        end
        
        % Evitamos minúsculos errores flotantes
        T1 = max(0, T1); T2 = max(0, T2); T3 = max(0, T3);
        
        % =================================================================
        % ACTUALIZAR VECTORES EN PANTALLA
        % =================================================================
        % La flecha azul apunta hacia donde querés ir
        set(h_mov, 'UData', x_norm, 'VData', y_norm);
        
        % Los pads salen desde el centro hacia sus respectivos ángulos
        set(h_pad1, 'UData', T1*cos(0),       'VData', T1*sin(0));
        set(h_pad2, 'UData', T2*cos(2*pi/3),  'VData', T2*sin(2*pi/3));
        set(h_pad3, 'UData', T3*cos(4*pi/3),  'VData', T3*sin(4*pi/3));
        
        % Actualizar recuadro de texto
        txt = sprintf(' Azimut Deseado: %.0f° \n\n Tensión Cables:\n Pad 1 (0°):   %.2f N \n Pad 2 (120°): %.2f N \n Pad 3 (240°): %.2f N ', ...
                      theta_mov*180/pi, T1, T2, T3);
        set(h_text, 'String', txt);
    end
end