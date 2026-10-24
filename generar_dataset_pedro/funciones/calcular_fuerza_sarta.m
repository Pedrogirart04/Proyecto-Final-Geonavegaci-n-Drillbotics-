function P_vector = calcular_fuerza_sarta(matriz_datos, col_deflexion, col_z)
    % CALCULAR_Y_GRAFICAR: Lee deflexiones y posiciones z de una matriz,
    % calcula la fuerza P (en N) y grafica P en función de z.
    %
    % Uso: P_vector = calcular_y_graficar(mi_matriz, 2, 1)

    % 1. Propiedades geométricas y del material del tubo de aluminio
    D = 12.7;        % Diámetro exterior [mm]
    d = 9.5;         % Diámetro interior [mm]
    L = 770;         % Longitud del tubo [mm]
    E = 69000;       % Módulo de elasticidad [MPa]

    % 2. Cálculo del momento de inercia y rigidez (k)
    I = (pi / 64) * (D^4 - d^4);
    k = (3 * E * I) / (L^3);

    % 3. Extracción de los datos (Acá aplicamos tu idea)
    deflexiones = matriz_datos(:, col_deflexion); % Extrae la columna de deflexión (tu 'x')
    valores_z   = matriz_datos(:, col_z);         % Extrae la columna de 'z'

    % 4. Cálculo vectorizado de las fuerzas
    P_vector = k .* deflexiones;

    % 5. Generación del gráfico
    figure; % Abre una ventana nueva
    
    % plot(eje_horizontal, eje_vertical, ...)
    plot(valores_z, P_vector, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', '#0072BD');
    
    % Detalles estéticos del gráfico
    grid on;
    xlabel('Posición z', 'FontWeight', 'bold');
    ylabel('Fuerza P aplicada [N]', 'FontWeight', 'bold');
    title('Fuerza necesaria vs. Posición z', 'FontSize', 12);
end