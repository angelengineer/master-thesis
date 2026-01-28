function x_next = armStateTransitionFcn(x, inputs)
    % Estado: x = [theta; theta_dot; r]
    % Inputs: [tau, mass, dt]
    % 
    % Dinámica:
    %   theta_ddot = (tau + m*g*r*cos(theta)) / (m*r^2)
    
    tau = inputs(1);    % Torque aplicado [Nm]
    mass = inputs(2);   % Masa conocida [kg]
    dt = inputs(3);     % Tiempo de muestreo [s]
    
    g = 9.81;           % Gravedad [m/s^2]
    
    % Estado actual
    theta = x(1);       % Ángulo [rad]
    theta_dot = x(2);   % Velocidad angular [rad/s]
    r = x(3);           % Distancia al CoM [m] - parámetro a estimar
    
    % Evitar división por cero
    if abs(r) < 1e-6
        r = sign(r) * 1e-6;
    end
    
    % Dinámica del sistema (integración Euler)
    theta_ddot = (tau + mass * g * r * cos(theta)) / (mass * r^2);
    
    % Siguiente estado
    x_next = zeros(3,1);
    x_next(1) = theta + theta_dot * dt;              % Posición angular
    x_next(2) = theta_dot + theta_ddot * dt;         % Velocidad angular
    x_next(3) = r;                                    % r permanece constante
end