function x_next = armSimpleTransitionFcn(x, inputs)

    theta     = x(1);
    theta_dot = x(2);
    p         = x(3);   % p = m*r

    tau = inputs(1);
    dt  = inputs(2);

    g = 9.81;
    J = 1;   % normalizado o aproximado (no crítico)

    theta_ddot = (tau + p * g * cos(theta)) / J;

    x_next = zeros(3,1);
    x_next(1) = theta + theta_dot * dt;
    x_next(2) = theta_dot + theta_ddot * dt;
    x_next(3) = p;   % parámetro constante
end
