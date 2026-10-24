function Matriz_Pads = descomponer_fuerzas_pads_silent(Matriz_Trayectoria, Fuerza_final)
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
    
end