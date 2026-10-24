%% =========================================================================
% VISUALIZACIÓN DEL ENTORNO DE PERFORACIÓN: PIEDRA, CONO Y TRAYECTORIA
% Proyecto Drillbotics - ITBA
% =========================================================================
clear; clc; close all;

%% 1. PARÁMETROS GEOMÉTRICOS (en centímetros)
% Dimensiones de la piedra (Ajustar Ly a 30 si la base es rectangular)
Lx = 30;   % Ancho en X
Ly = 60;   % Profundidad en Y 
Lz = 60;   % Altura (Profundidad Z)

% Dimensiones del Cono de competencia
R_base = 10; % Radio en el fondo de la piedra (Z = 60)
% La punta del cono estará en el centro de la superficie (Z = 0)

%% 2. CONFIGURACIÓN DE LA FIGURA
fig = figure('Name', 'Entorno de Perforación Drillbotics', 'Color', 'w', ...
    'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.7 0.8]);
hold on; grid on; axis equal;
view(35, 30);
set(gca, 'ZDir', 'reverse'); % Invertimos Z para que la profundidad vaya hacia abajo

%% 3. DIBUJO DE LA PIEDRA SEMITRANSPARENTE
% Definimos los vértices del bloque (Centro en X=0, Y=0, Z arranca en 0)
x_min = -Lx/2; x_max = Lx/2;
y_min = -Ly/2; y_max = Ly/2;
z_min = 0;     z_max = Lz;

vertices_piedra = [
    x_min y_min z_min;  x_max y_min z_min;  
    x_max y_max z_min;  x_min y_max z_min;
    x_min y_min z_max;  x_max y_min z_max;  
    x_max y_max z_max;  x_min y_max z_max
];

caras_piedra = [
    1 2 3 4; % Cara Superior
    5 6 7 8; % Cara Inferior
    1 2 6 5; % Frontal
    2 3 7 6; % Derecha
    3 4 8 7; % Trasera
    4 1 5 8  % Izquierda
];

% Dibujamos el bloque con 'patch' (Color gris, transparencia alpha)
patch('Vertices', vertices_piedra, 'Faces', caras_piedra, ...
      'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.15, ...
      'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);

%% 4. DIBUJO DEL CONO DE RESTRICCIÓN
% Generamos un cilindro paramétrico y lo convertimos en cono (radio 0 a R_base)
[Xc, Yc, Zc] = cylinder([0, R_base], 60);
Zc = Zc * Lz; % Escalamos Z de 0 a 60 cm

surf(Xc, Yc, Zc, 'FaceColor', [0.9 0.8 0.2], 'FaceAlpha', 0.2, ...
     'EdgeColor', 'none');

%% 5. GENERACIÓN DE LA TRAYECTORIA TIPO 'S' DENTRO DEL CONO
N_puntos = 200;
z_traj = linspace(0, Lz, N_puntos);

% Para que la curva sea S y no salga del cono, limitamos su amplitud al radio 
% permitido en cada profundidad: R_permitido(z) = (R_base/Lz) * z
R_permitido = (R_base / Lz) * z_traj;

% Ecuaciones paramétricas para la forma de 'S' en 3D
% Usamos el 80% del radio permitido para que no roce las paredes del cono
x_traj = (0.8 .* R_permitido) .* sin(2 * pi * (z_traj / Lz)); 
y_traj = (0.4 .* R_permitido) .* sin(2 * pi * (z_traj / Lz) + pi/4); 

plot3(x_traj, y_traj, z_traj, 'b-', 'LineWidth', 3);

%% 6. WAYPOINTS (LOS 3 PUNTOS DE LA COMPETENCIA)
% Seleccionamos 3 puntos representativos a lo largo de la trayectoria
z_waypoints = [15, 35, 55]; % Profundidades objetivo
x_wp = (0.8 * (R_base/Lz) * z_waypoints) .* sin(2 * pi * (z_waypoints / Lz));
y_wp = (0.4 * (R_base/Lz) * z_waypoints) .* sin(2 * pi * (z_waypoints / Lz) + pi/4);

% Graficamos los Waypoints
scatter3(x_wp, y_wp, z_waypoints, 120, 'ro', 'filled', 'MarkerEdgeColor', 'k');

% Agregamos etiquetas a los puntos
for i = 1:3
    texto_wp = sprintf('  P%d', i);
    text(x_wp(i), y_wp(i), z_waypoints(i), texto_wp, 'FontSize', 10, 'FontWeight', 'bold');
end

%% 7. ACOTACIÓN DE LA PIEDRA (LÍNEAS DE DIMENSIONES)
offset = 5; % Separación de las cotas respecto a la piedra

% Cota Ancho (Eje X)
plot3([-Lx/2, Lx/2], [-Ly/2-offset, -Ly/2-offset], [0, 0], 'k|-', 'LineWidth', 1.5, 'MarkerSize', 6);
text(0, -Ly/2-offset-3, 0, sprintf('%d cm', Lx), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

% Cota Profundidad (Eje Y)
plot3([Lx/2+offset, Lx/2+offset], [-Ly/2, Ly/2], [0, 0], 'k|-', 'LineWidth', 1.5, 'MarkerSize', 6);
text(Lx/2+offset+3, 0, 0, sprintf('%d cm', Ly), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

% Cota Altura (Eje Z)
plot3([Lx/2+offset, Lx/2+offset], [Ly/2+offset, Ly/2+offset], [0, Lz], 'k|-', 'LineWidth', 1.5, 'MarkerSize', 6);
text(Lx/2+offset+3, Ly/2+offset+3, Lz/2, sprintf('%d cm', Lz), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

%% 8. RETOQUES FINALES DE LA GRÁFICA
title('Entorno Físico - Competencia Drillbotics', 'FontSize', 14);
xlabel('Eje X [cm]');
ylabel('Eje Y [cm]');
zlabel('Profundidad Z [cm]');
legend('Bloque de Piedra', 'Cono de Límite (R=10cm)', 'Trayectoria Tipo S', 'Waypoints Objetivo', 'Location', 'northeast');

% Ajuste de límites para que las cotas no queden cortadas
xlim([-Lx/2-10, Lx/2+15]);
ylim([-Ly/2-15, Ly/2+15]);
zlim([-5, Lz+5]);

% Luces para darle efecto 3D al cono y bloque
camlight('headlight');
lighting gouraud;