function x_next = f_ext(x, u)
    % x = [pos; vel; theta; omega; Fext] (5 estados)
    % u = fuerza de control del MPC (escalar)
    dT = 0.05;
    % Parámetros (DEBEN COINCIDIR con tu planta real)
    M = 0.5;   % Masa del carro [kg] - AJUSTA
    m = 0.2;   % Masa del péndulo [kg] - AJUSTA
    l = 0.3;   % Longitud [m] - AJUSTA
    g = 9.81;  % Gravedad
    b = 0.1;   % Fricción
    
    % Extraer estados
    pos   = x(1);
    vel   = x(2);
    theta = x(3);  % Ángulo (0 = colgando, pi = invertido)
    omega = x(4);
    Fext  = x(5);  % Fuerza externa a estimar
    
    % Fuerza de control
    F = u;
    
    % Ecuaciones dinámicas CORRECTAS
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    total_mass = M + m;
    den = M + m*sin_theta^2;
    
    % Aceleraciones
    dpos   = vel;
    dvel   = (F + Fext - m*l*omega^2*sin_theta + m*g*sin(theta)*cos(theta) - b*vel) / den;
    dtheta = omega;
    domega = (g*sin_theta*(total_mass) - cos_theta*(F + Fext - b*vel) - m*l*omega^2*sin_theta*cos_theta) / (l*den);
    
    % Dinámica de Fext: Modelamos como constante con algo de ruido
    % ESTO ES CLAVE: permite que EKF actualice Fext
    dFext = 0;  % Asumimos constante entre muestras
    
    % Integración Euler
    x_next = x + [dpos; dvel; dtheta; domega; dFext] * dT;
end