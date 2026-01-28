function T = rotx(angle)
%ROTX Matriz de transformación homogénea para rotación alrededor del eje X
%
% Input:
%   angle - Ángulo de rotación en radianes
%
% Output:
%   T - Matriz de transformación homogénea 4x4

% Validar entrada
if ~isscalar(angle)
    error('El ángulo debe ser un escalar');
end

% Calcular seno y coseno
c = cos(angle);
s = sin(angle);

% Matriz de rotación homogénea alrededor del eje X
T = [1,  0,  0,  0;
     0,  c, -s,  0;
     0,  s,  c,  0;
     0,  0,  0,  1];

% Simplificar si es simbólico
if isa(T, 'sym')
    T = simplify(T);
end