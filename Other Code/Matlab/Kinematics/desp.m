function T = desp(vector)
%DESP Matriz de transformación homogénea para traslación pura
%
% Input:
%   vector - Vector de traslación [dx, dy, dz] o [dx, dy] (dz=0)
%
% Output:
%   T - Matriz de transformación homogénea 4x4

% Validar y procesar entrada
if length(vector) == 2
    dx = vector(1);
    dy = vector(2);
    dz = 0;
elseif length(vector) == 3
    dx = vector(1);
    dy = vector(2);
    dz = vector(3);
else
    error('El vector debe tener 2 o 3 elementos');
end

% Matriz de traslación homogénea
T = [1, 0, 0, dx;
     0, 1, 0, dy;
     0, 0, 1, dz;
     0, 0, 0, 1];

% Simplificar si es simbólico
if isa(T, 'sym')
    T = simplify(T);
end
end