function T = roty(angle)
%ROTY Matriz de transformación homogénea para rotación alrededor del eje Y
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

% Matriz de rotación homogénea alrededor del eje Y
T = [ c,  0,  s,  0;
      0,  1,  0,  0;
     -s,  0,  c,  0;
      0,  0,  0,  1];

% Simplificar si es simbólico
if isa(T, 'sym')
    T = simplify(T);
end
end