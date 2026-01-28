function x_next = myStateTransitionFcn(x, inputs)
    % x = [posición; velocidad; masa]
    % inputs = [F_aplicada, dt, pitch]
    
    F = inputs(1);
    dt = inputs(2);
    beta = inputs(3);  % Pitch (entrada externa)
    g = 9.81;
    
    x_next = zeros(3,1);
    x_next(1) = x(1) + x(2)*dt;
    x_next(2) = x(2) + (F/x(3) - g*cos(beta))*dt;
    x_next(3) = x(3);  % Masa constante
end