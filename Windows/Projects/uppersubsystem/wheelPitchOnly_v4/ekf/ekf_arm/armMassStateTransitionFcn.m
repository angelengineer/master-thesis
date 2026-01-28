function x_next = armMassStateTransitionFcn(x, inputs)
    % Estado EXTENDIDO: x = [theta; theta_dot; r; mass]
    % Inputs: [tau, dt]
    
    tau = inputs(1);    % Torque aplicado [Nm]
    dt = inputs(2);     % Tiempo de muestreo [s]
    g = 9.81;           % Gravedad [m/s^2]
    
    % Estado actual
    theta = x(1);       % Ángulo [rad]
    theta_dot = x(2);   % Velocidad angular [rad/s]
    r = x(3);           % Distancia al CoM [m]
    mass = x(4);        % Masa [kg] - ahora es parte del estado
    
    % Evitar división por cero y valores negativos
    r = max(abs(r), 1e-6) * sign(r);
    mass = max(mass, 0.01);  % Masa mínima física
    
    % Dinámica del sistema
    theta_ddot = (tau + mass * g * r * cos(theta)) / (mass * r^2);
    
    % Siguiente estado
    x_next = zeros(4,1);
    x_next(1) = theta + theta_dot * dt;              
    x_next(2) = theta_dot + theta_ddot * dt;         
    x_next(3) = r;      % r permanece constante (parámetro)
    x_next(4) = mass;   % masa permanece constante (parámetro)
end