function samples = lhs_manual(N, d)
% Latin Hypercube Sampling manual (no requiere Statistics Toolbox).
% Devuelve una matriz N x d con valores en [0,1]: cada columna tiene
% exactamente un valor en cada uno de los N intervalos de ancho 1/N,
% con el orden de los intervalos permutado al azar por columna.
    samples = zeros(N, d);
    for j = 1:d
        perm = randperm(N)';          % orden aleatorio de los N intervalos
        samples(:, j) = (perm - rand(N, 1)) / N;  % jitter dentro de cada intervalo
    end
end