function [cdg, I_total] = compute_cdg_and_inertia(m, x, z, I_y)
%COMPUTE_CDG_AND_INERTIA
%   Calcula el centro de gravedad total y la inercia equivalente
%   respecto al eje Y que pasa por el CDG total (teorema de Steiner)
%
% Inputs:
%   m   : [Nx1] masas de cada cuerpo
%   x   : [Nx1] posición x del CDG de cada cuerpo
%   z   : [Nx1] posición z del CDG de cada cuerpo
%   I_y : [Nx1] inercia de cada cuerpo respecto a su propio CDG (eje Y)
%
% Outputs:
%   cdg     : [1x2] [x_G, z_G] centro de gravedad total
%   I_total : escalar, inercia total respecto al CDG global (eje Y)

    % Comprobaciones básicas
    assert(numel(m) == numel(x) && numel(x) == numel(z) ...
        && numel(z) == numel(I_y), ...
        'Todos los vectores deben tener la misma longitud');

    % Masa total
    M = sum(m);

    % Centro de gravedad total
    x_G = sum(m .* x) / M;
    z_G = sum(m .* z) / M;

    cdg = [x_G, z_G];

    % Distancia al CDG global
    dx = x - x_G;
    dz = z - z_G;
    d2 = dx.^2 + dz.^2;

    % Inercia total usando Steiner
    I_total = sum(I_y + m .* d2);
end
