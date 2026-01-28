function y = armMeasurementUKF(x)
    % UKF requiere: (estado, entrada)
    % Medimos solo el ángulo theta
    y = x(1);
end