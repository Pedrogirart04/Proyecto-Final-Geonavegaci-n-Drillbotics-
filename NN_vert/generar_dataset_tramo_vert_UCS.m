%% =========================================================================
% GENERAR_DATASET_UCS - Dataset de entrenamiento para la red de estimación
%                        de UCS (Fase C): (ROP, RPM, WOB) -> UCS
% =========================================================================
% Se busca estimar el UCS de la roca a partir de variables medibles en 
% perforación real (ROP, RPM,% WOB), para poder usarlo como input de 
% optimización de parámetros de perforación en competencia.
%
% Dataset SINTÉTICO (este script): se genera con el modelo analítico de
% fuerza axial de corte (misma familia que Perneder, caso vertical/no
% direccional, sin curvatura):
%
%     W = pi * a * UCS * ROP / (2 * RPM)
%
% donde W = WOB [N], a = radio del trépano [mm], UCS = epsilon [MPa],
% ROP [mm/min], RPM [rpm]. 
%
% A diferencia del dataset real (n probetas de UCS conocido x n^2
% combinaciones de ROP,RPM cada una), acá se usa Latin Hypercube Sampling
% (LHS) sobre (ROP, RPM, UCS) para cubrir el cubo de forma pareja con
% muestras continuas — ver fase-c-plan.md para la justificación.
%
% Salida CSV: UCS_MPa | ROP_mm_min | RPM | WOB_N | probeta_id
%
% probeta_id: en este dataset sintético cada fila es una muestra LHS
% independiente (UCS continuo, no hay "probetas" repetidas), así que
% probeta_id = índice de fila (grupos triviales de tamaño 1). Cuando se
% reemplace por datos reales, probeta_id se repetirá n^2 veces por cada
% probeta física real — el split agrupado en datos.py funciona IGUAL sin
% tocar código, porque ya está escrito en términos de grupos.
% =========================================================================
clear; clc; close all;

%% 1. PARÁMETROS DE GENERACIÓN
N_muestras = 1000;      % <<<< AJUSTAR SI CAMBIA: cantidad de puntos sintéticos
semilla = 42;
rng(semilla);           % Reproducibilidad

%% 2. RANGOS DE LAS VARIABLES (sorteadas vía LHS)
ROP_range = [5, 30];       % [mm/min]  
RPM_range = [1000, 2000];  % [rpm]     
UCS_range = [20, 30];      % [MPa]    

%% 3. PARÁMETROS FIJOS DEL SISTEMA
a = 1.5 * 25.4 / 2;   % Radio del trépano [mm] (igual que Fase B / Main.m)

%% 4. LATIN HYPERCUBE SAMPLING DE (ROP, RPM, UCS)
lhs_samples = lhs_manual(N_muestras, 3);
ROP_vals = ROP_range(1) + lhs_samples(:,1) * diff(ROP_range);
RPM_vals = RPM_range(1) + lhs_samples(:,2) * diff(RPM_range);
UCS_vals = UCS_range(1) + lhs_samples(:,3) * diff(UCS_range);

%% 5. CALCULAR WOB (fórmula analítica directa, sin trayectoria ni loop)
WOB_vals = pi * a * UCS_vals .* ROP_vals ./ (2 * RPM_vals);

%% 6. ARMAR MATRIZ Y EXPORTAR A CSV
probeta_id = (1:N_muestras)';   % grupos triviales (ver nota arriba)

datos = [UCS_vals, ROP_vals, RPM_vals, WOB_vals, probeta_id];

nombre_archivo = 'dataset_UCS_sintetico.csv';   % <<<< AJUSTAR RUTA/NOMBRE SI HACE FALTA
header = {'UCS_MPa', 'ROP_mm_min', 'RPM', 'WOB_N', 'probeta_id'};
fid = fopen(nombre_archivo, 'w');
fprintf(fid, '%s,%s,%s,%s,%s\n', header{:});
fclose(fid);
dlmwrite(nombre_archivo, datos, '-append', 'delimiter', ',', 'precision', '%.8f');

%% 7. RESUMEN
fprintf('=========================================\n');
fprintf('  RESUMEN DEL DATASET UCS (SINTÉTICO)\n');
fprintf('=========================================\n');
fprintf('  Muestras generadas:  %d\n', N_muestras);
fprintf('  ROP rango:           [%.2f, %.2f] mm/min\n', min(ROP_vals), max(ROP_vals));
fprintf('  RPM rango:           [%.1f, %.1f]\n', min(RPM_vals), max(RPM_vals));
fprintf('  UCS rango:           [%.2f, %.2f] MPa\n', min(UCS_vals), max(UCS_vals));
fprintf('  WOB rango:           [%.2f, %.2f] N\n', min(WOB_vals), max(WOB_vals));
fprintf('  Archivo guardado:    %s\n', nombre_archivo);
fprintf('=========================================\n');