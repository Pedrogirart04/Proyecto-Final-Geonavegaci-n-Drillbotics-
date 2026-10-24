function Matriz_Pads = descomponer_fuerzas_pads(Matriz_Trayectoria, Fuerza_final)
%% =========================================================================
% DESCOMPOSICIÓN VECTORIAL POR AZIMUT (PUSH-THE-BIT)
% =========================================================================
    
    Z_global = Matriz_Trayectoria(:, 3);
    
    % EXTRACCIÓN DIRECTA DEL AZIMUT DE TU GENERADOR (Columna 6)
    % Este valor es robusto, suavizado y no sufre inversiones numéricas.
    Phi_deg = Matriz_Trayectoria(:, 6); 
    N_puntos = length(Z_global);

    T1 = zeros(N_puntos, 1);
    T2 = zeros(N_puntos, 1);
    T3 = zeros(N_puntos, 1);

    %% LÓGICA TRIGONOMÉTRICA PUSH-THE-BIT
    for i = 1:N_puntos
        F = Fuerza_final(i);
        
        % Ignorar zonas rectas donde no hay fuerza requerida
        if F < 1e-2
            continue;
        end
        
        % 1. Hacia dónde vamos (Ángulo de movimiento en radianes)
        theta_mov = Phi_deg(i) * (pi/180);
        
        % 2. Hacia dónde empujamos (Pared opuesta -> sumamos pi)
        theta_push = mod(theta_mov + pi, 2*pi);
        th = theta_push;
        
        % 3. Reparto de fuerzas por cuadrantes
        if th >= 0 && th < (2*pi/3)
            % SECTOR 1: Trabajan Pad 1 y Pad 2
            T1(i) = F * (cos(th) + (1/sqrt(3))*sin(th));
            T2(i) = F * ((2/sqrt(3))*sin(th));
            T3(i) = 0;
            
        elseif th >= (2*pi/3) && th < (4*pi/3)
            % SECTOR 2: Trabajan Pad 2 y Pad 3 (Para moverse a +X, entra acá)
            T1(i) = 0;
            T2(i) = F * (-cos(th) + (1/sqrt(3))*sin(th));
            T3(i) = F * (-cos(th) - (1/sqrt(3))*sin(th));
            
        else
            % SECTOR 3: Trabajan Pad 3 y Pad 1
            T1(i) = F * (cos(th) - (1/sqrt(3))*sin(th));
            T2(i) = 0;
            T3(i) = F * (-(2/sqrt(3))*sin(th));
        end
    end

    % Filtro de seguridad por redondeos
    T1(T1 < 1e-3) = 0; T2(T2 < 1e-3) = 0; T3(T3 < 1e-3) = 0;

    Matriz_Pads = [T1, T2, T3];
    
    %% GRÁFICO: SUBPLOTS EN COLUMNA
    fig_pads = figure('Name', 'Perfil de Actuación por Azimut (Subplots)', 'Color', 'w', ...
                      'Units', 'normalized', 'Position', [0.1 0.1 0.5 0.8]);
    
    % Tonos RGB para compatibilidad
    c1 = [0.6350, 0.0780, 0.1840]; % Rojo
    c2 = [0.4660, 0.6740, 0.1880]; % Verde
    c3 = [0.8500, 0.3250, 0.0980]; % Naranja

    % Calculamos el límite máximo de Y para que todos los gráficos tengan la misma escala
    max_F = max([T1; T2; T3]);
    if max_F < 1e-3
        max_F = 1; % Para evitar errores si el arreglo es todo ceros
    end
    y_limits = [0, max_F * 1.1]; % Le damos un 10% de margen superior

    % Subplot 1: Pad 1
    ax1 = subplot(3, 1, 1);
    plot(Z_global, T1, 'Color', c1, 'LineWidth', 2); grid on;
    ylabel('Fuerza [N]', 'FontWeight', 'bold');
    title('Actuación Pad 1 (0^\circ - Eje +X)', 'FontSize', 11);
    ylim(ax1, y_limits);
    
    % Subplot 2: Pad 2
    ax2 = subplot(3, 1, 2);
    plot(Z_global, T2, 'Color', c2, 'LineWidth', 2); grid on;
    ylabel('Fuerza [N]', 'FontWeight', 'bold');
    title('Actuación Pad 2 (120^\circ)', 'FontSize', 11);
    ylim(ax2, y_limits);
    
    % Subplot 3: Pad 3
    ax3 = subplot(3, 1, 3);
    plot(Z_global, T3, 'Color', c3, 'LineWidth', 2); grid on;
    xlabel('Profundidad Z [mm]', 'FontWeight', 'bold');
    ylabel('Fuerza [N]', 'FontWeight', 'bold');
    title('Actuación Pad 3 (240^\circ)', 'FontSize', 11);
    ylim(ax3, y_limits);

    % Vinculamos los ejes X para que al hacer zoom en uno, se muevan los tres juntos
    linkaxes([ax1, ax2, ax3], 'x');
end