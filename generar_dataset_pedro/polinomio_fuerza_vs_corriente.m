%% =========================================================================
% SCRIPT SECUNDARIO - AJUSTE DE CURVA CORRIENTE VS CARGA (SERVO)
% =========================================================================
clear; clc; close all;

% 1. Carga de los datos de ensayo extraídos de la imagen
Kg = [0; 1.839; 3.109; 5.404; 6.424; 7.444; 8.463; 9.511; 10.482; 11.513; 12.525; 13.536; 14.388; 15.408; 16.368];
A  = [0.073; 0.085; 0.20; 0.30; 0.35; 0.43; 0.62; 0.83; 0.89; 1.00; 1.18; 1.30; 1.45; 1.93; 2.00];

% 2. Ajustes polinómicos (polyfit devuelve los coeficientes con máxima precisión)
p2 = polyfit(Kg, A, 2);
p3 = polyfit(Kg, A, 3);
p4 = polyfit(Kg, A, 4);

% 3. Evaluación de las curvas para graficar suave
Kg_fino = linspace(0, 18, 200); % Me extiendo un poco más para ver cómo se comportan fuera del rango
A2_fino = polyval(p2, Kg_fino);
A3_fino = polyval(p3, Kg_fino);
A4_fino = polyval(p4, Kg_fino);

% 4. Reporte en consola de los coeficientes listos para copiar
fprintf('======================================================\n');
fprintf(' COEFICIENTES PARA PEGAR EN EL SCRIPT MAIN (Elegí uno)\n');
fprintf('======================================================\n');
fprintf('%% Opción 1: Polinomio Grado 2 (Más estable, menos preciso)\n');
fprintf('coef_I = [%.8e, %.8e, %.8e];\n\n', p2);

fprintf('%% Opción 2: Polinomio Grado 3 (RECOMENDADO - Buen balance)\n');
fprintf('coef_I = [%.8e, %.8e, %.8e, %.8e];\n\n', p3);

fprintf('%% Opción 3: Polinomio Grado 4 (El de Excel - Precisión máxima en los puntos)\n');
fprintf('coef_I = [%.8e, %.8e, %.8e, %.8e, %.8e];\n', p4);
fprintf('======================================================\n');

% 5. Gráfico comparativo
figure('Name', 'Comparación de Ajustes Polinómicos', 'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0.2 0.2 0.6 0.6]);
plot(Kg, A, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'DisplayName', 'Datos de Ensayo'); hold on; grid on;

%plot(Kg_fino, A2_fino, 'LineWidth', 2, 'DisplayName', 'Ajuste Grado 2');
plot(Kg_fino, A3_fino, 'LineWidth', 2.5, 'DisplayName', 'Ajuste Grado 3');
%plot(Kg_fino, A4_fino, 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Ajuste Grado 4 (Excel)');

xlabel('Carga Equivalente [Kg]', 'FontSize', 12);
ylabel('Corriente del Servo [A]', 'FontSize', 12);
title('Curva de Caracterización del Servomotor (Corriente vs Carga)');
legend('Location', 'northwest', 'FontSize', 11);
xlim([0, 18]); ylim([0, 2.5]);