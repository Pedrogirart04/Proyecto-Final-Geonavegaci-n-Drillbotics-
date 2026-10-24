function Matriz_Salida = generar_trayectoria_v2_silent(P2, P3, L_bit)
    %% =========================================================================
    % FUNCIÓN GENERADORA DE TRAYECTORIA — v2: tramo recto inicial forzado vertical
    % P1 ya no se recibe como parámetro: se calcula como [0,0,L_bit], igual
    % que en trayectoria.py (TrayectoriaIdeal). Así el tramo recto P0->P1
    % es siempre vertical, sin importar dónde caigan P2/P3.
    % =========================================================================
    if nargin < 3
        L_bit = 120; % [mm]
    end

    % 1. PARÁMETROS FÍSICOS Y WAYPOINTS BÁSICOS [EN MILÍMETROS]
    R_min = 186.43; % Radio de curvatura mínimo admisible [mm]

    P0 = [0, 0, 0];
    P1 = [0, 0, L_bit];
    P_global = [P0; P1; P2; P3]';

    % La curva calculada por spline arranca estrictamente en P1
    P_curve = [P1; P2; P3]';

    % Longitud del tramo recto inicial = L_bit (fija, ya no se calcula con norm)
    L_recto = L_bit;

    % 2. PARAMETRIZACIÓN POR DISTANCIA ACUMULADA (SOLO TRAMO CURVO)
    dists_curve = sqrt(sum(diff(P_curve, 1, 2).^2, 1));
    t_waypoints_curve = [0, cumsum(dists_curve)];

    Total_S = L_recto + t_waypoints_curve(end);
    N_puntos = 1200;
    t_fino = linspace(0, Total_S, N_puntos);

    % 3. INTERPOLACIÓN MIXTA (RECTA VERTICAL P0->P1 + SPLINE SUJETO P1->P3)
    X = zeros(1, N_puntos);
    Y = zeros(1, N_puntos);
    Z = zeros(1, N_puntos);

    idx_recto = t_fino <= L_recto;
    idx_curvo = t_fino > L_recto;

    % TRAMO RECTO INICIAL: estrictamente vertical
    X(idx_recto) = 0;
    Y(idx_recto) = 0;
    Z(idx_recto) = t_fino(idx_recto);

    % TRAMO CURVO: la spline opera únicamente desde P1 en adelante
    v_start = (P1 - P0)' / L_recto;  % = [0,0,1], tangente vertical
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

    dX(idx_recto) = 0;
    dY(idx_recto) = 0;
    dZ(idx_recto) = 1;

    dX(idx_curvo) = gradient(X(idx_curvo), t_fino(idx_curvo));
    dY(idx_curvo) = gradient(Y(idx_curvo), t_fino(idx_curvo));
    dZ(idx_curvo) = gradient(Z(idx_curvo), t_fino(idx_curvo));

    ds = sqrt(dX.^2 + dY.^2 + dZ.^2);
    S_perforada = [0, cumsum(ds(1:end-1) .* diff(t_fino))];
    Theta_ref = acos(dZ ./ ds);
    Theta_deg = Theta_ref * (180/pi);

    delta_theta_deg = diff(Theta_deg);
    Theta_deg_acumulado = [0, cumsum(abs(delta_theta_deg))];

    % 5. CÁLCULO DE AZIMUT POR VECTORES SECANTES
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

    % 6. ANÁLISIS DE CURVATURA ANALÍTICA 3D
    ddX = zeros(1, N_puntos); ddY = zeros(1, N_puntos); ddZ = zeros(1, N_puntos);

    ddX(idx_curvo) = gradient(dX(idx_curvo), t_fino(idx_curvo));
    ddY(idx_curvo) = gradient(dY(idx_curvo), t_fino(idx_curvo));
    ddZ(idx_curvo) = gradient(dZ(idx_curvo), t_fino(idx_curvo));

    Numerador_K = sqrt((dY.*ddZ - dZ.*ddY).^2 + (dZ.*ddX - dX.*ddZ).^2 + (dX.*ddY - dY.*ddX).^2);
    Curvatura = Numerador_K ./ (ds.^3);
    Curvatura(idx_recto) = 0;

    Max_Curvatura = max(Curvatura(idx_curvo));
    Min_Radio_Real = 1 / (Max_Curvatura + 1e-12);

    Radio_Evolucion = 1 ./ (Curvatura + 1e-12);
    Radio_Evolucion(Radio_Evolucion > 10000) = 10000;

    % 7. CONSTRUCCIÓN DE MATRIZ DE SALIDA (N x 6)
    Matriz_Salida = [X(:), Y(:), Z(:), Radio_Evolucion(:), Theta_deg(:), Phi_deg_absoluto(:)];
end