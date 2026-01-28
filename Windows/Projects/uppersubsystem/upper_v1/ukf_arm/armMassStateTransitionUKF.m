function x_next = armMassStateTransitionUKF(x, inputs)
    % x = [theta; theta_dot; Jeq]
    % inputs = [tau; dt]

    tau = inputs(1);
    dt  = inputs(2);

    g = 9.81;
    r_nom = 0.33;   % distancia nominal fija

    theta     = x(1);
    theta_dot = x(2);
    Jeq       = max(x(3), 1e-4); % evitar valores no físicos

    theta_ddot = tau/Jeq + g*cos(theta)/r_nom;

    x_next = zeros(3,1);
    x_next(1) = theta + theta_dot*dt;
    x_next(2) = theta_dot + theta_ddot*dt;
    x_next(3) = Jeq;  % parámetro lento
end
