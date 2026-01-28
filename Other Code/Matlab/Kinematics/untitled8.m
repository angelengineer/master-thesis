syms Lb d2 rw theta1 La theta3
expresion = 3.7e-33*Lb + 3.7e-33*d2 + 1.0*rw + 1.0*cos(theta1)*(Lb + d2) - 1.0*La*(1.0*cos(theta3)*sin(theta1) + 6.1e-17*sin(theta3)*(6.1e-17*cos(theta1) - 6.1e-17) + sin(theta3)*(1.0*cos(theta1) + 3.7e-33));

% Definir la tolerancia (p. ej., 1e-16, que es la precisión de la máquina)
tolerancia = 1e-15;

% Obtener los coeficientes
coeficientes = symvar(expresion); % Obtiene las variables simbólicas

% Recorrer la expresión y buscar coeficientes pequeños (Esto puede ser complejo para expresiones anidadas)
% La forma más robusta es usar el método 'coeffs' y sustituir:
[C, T] = coeffs(expresion); % C son los coeficientes, T son los términos

% Encontrar los coeficientes cuyo valor absoluto es menor que la tolerancia
indices_pequenos = abs(double(C)) < tolerancia;

% Crear una lista de sustituciones: los coeficientes pequeños se reemplazan por 0
sustituciones = C(indices_pequenos) == 0;

% Aplicar las sustituciones
expresion_limpia = subs(expresion, sustituciones);

% Una simplificación final para consolidar (0*algo = 0)
expresion_limpia = simplify(expresion_limpia);